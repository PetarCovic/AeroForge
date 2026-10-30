#include <hil/HilSerialWorker.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace aeroforge::hil
{
    HilSerialWorker::HilSerialWorker(
        std::unique_ptr<HilByteDevice> device)
        : device_(std::move(device))
    {
        if (!device_)
        {
            throw std::invalid_argument(
                "HIL serial worker requires a byte device");
        }
    }

    HilSerialWorker::~HilSerialWorker()
    {
        stop();
    }

    bool HilSerialWorker::start()
    {
        if (thread_.joinable())
        {
            return false;
        }

        {
            std::lock_guard lock(rxMutex_);

            rxHead_ = 0;
            rxTail_ = 0;
            rxSize_ = 0;
        }

        {
            std::lock_guard lock(txMutex_);

            txLength_ = 0;
            writeState_ = WriteState::IDLE;
        }

        lastError_ = 0;
        receivedBytes_ = 0;
        transmittedBytes_ = 0;
        droppedReceiveBytes_ = 0;

        state_ = HilSerialWorkerState::STARTING;

        try
        {
            thread_ = std::jthread(
                [this](std::stop_token token)
                {
                    run(token);
                });
        }
        catch (...)
        {
            state_ = HilSerialWorkerState::STOPPED;
            return false;
        }

        return true;
    }

    void HilSerialWorker::stop()
    {
        if (thread_.joinable())
        {
            thread_.request_stop();
            thread_.join();
        }

        state_ = HilSerialWorkerState::STOPPED;

        {
            std::lock_guard lock(rxMutex_);

            rxHead_ = 0;
            rxTail_ = 0;
            rxSize_ = 0;
        }

        {
            std::lock_guard lock(txMutex_);

            txLength_ = 0;
            writeState_ = WriteState::IDLE;
        }
    }

    HilBytePort HilSerialWorker::makePort()
    {
        return HilBytePort{
            [this](std::span<std::uint8_t> output)
            {
                return popReceived(output);
            },
            [this](std::span<const std::uint8_t> input)
            {
                return requestWrite(input);
            }
        };
    }

    HilSerialWorkerState
        HilSerialWorker::state() const noexcept
    {
        return state_.load();
    }

    std::uint32_t
        HilSerialWorker::lastError() const noexcept
    {
        return lastError_.load();
    }

    std::uint64_t
        HilSerialWorker::receivedBytes() const noexcept
    {
        return receivedBytes_.load();
    }

    std::uint64_t
        HilSerialWorker::transmittedBytes() const noexcept
    {
        return transmittedBytes_.load();
    }

    std::uint64_t
        HilSerialWorker::droppedReceiveBytes() const noexcept
    {
        return droppedReceiveBytes_.load();
    }

    std::size_t HilSerialWorker::pushReceived(
        std::span<const std::uint8_t> input)
    {
        std::lock_guard lock(rxMutex_);

        const std::size_t accepted =
            std::min(input.size(), RX_CAPACITY - rxSize_);

        for (std::size_t i = 0; i < accepted; ++i)
        {
            rx_[rxTail_] = input[i];
            rxTail_ = (rxTail_ + 1) % RX_CAPACITY;
        }

        rxSize_ += accepted;

        return accepted;
    }

    std::size_t HilSerialWorker::popReceived(
        std::span<std::uint8_t> output)
    {
        if (state_ != HilSerialWorkerState::RUNNING)
        {
            return 0;
        }

        std::lock_guard lock(rxMutex_);

        const std::size_t count =
            std::min(output.size(), rxSize_);

        for (std::size_t i = 0; i < count; ++i)
        {
            output[i] = rx_[rxHead_];
            rxHead_ = (rxHead_ + 1) % RX_CAPACITY;
        }

        rxSize_ -= count;

        return count;
    }

    std::size_t HilSerialWorker::requestWrite(
        std::span<const std::uint8_t> input)
    {
        if (state_ != HilSerialWorkerState::RUNNING ||
            input.empty() ||
            input.size() > tx_.size())
        {
            return 0;
        }

        std::lock_guard lock(txMutex_);

        if (writeState_ == WriteState::COMPLETED)
        {
            // HilWireEndpoint must retry with the same
            // byte span until completion is acknowledged.
            if (input.size() != txLength_ ||
                !std::equal(
                    input.begin(),
                    input.end(),
                    tx_.begin()))
            {
                fail(0xFFFFFFFFu);
                return 0;
            }

            const std::size_t completed = txLength_;

            txLength_ = 0;
            writeState_ = WriteState::IDLE;

            return completed;
        }

        if (writeState_ != WriteState::IDLE)
        {
            // A write is already awaiting completion.
            return 0;
        }

        std::copy(
            input.begin(),
            input.end(),
            tx_.begin());

        txLength_ = input.size();
        writeState_ = WriteState::PENDING;

        // IMPORTANT: not yet written to the COM driver.
        // Returning zero makes HilWireEndpoint retry.
        return 0;
    }

    void HilSerialWorker::fail(std::uint32_t error)
    {
        lastError_ = error == 0 ? 0xFFFFFFFFu : error;
        state_ = HilSerialWorkerState::FAILED;
    }

    void HilSerialWorker::run(
        std::stop_token stopToken)
    {
        if (!device_->open())
        {
            fail(device_->lastError());
            device_->close();
            return;
        }

        state_ = HilSerialWorkerState::RUNNING;

        std::array<std::uint8_t, 256> incoming{};

        std::array<std::uint8_t,
            MAX_FRAMED_PACKET_SIZE> outbound{};

        std::size_t outgoingLength = 0;
        std::size_t outgoingOffset = 0;

        while (!stopToken.stop_requested() &&
            state_ == HilSerialWorkerState::RUNNING)
        {
            bool madeProgress = false;

            // ------------------------------------------------
            // Start an outstanding transmit request.
            // ------------------------------------------------
            if (outgoingLength == 0)
            {
                std::lock_guard lock(txMutex_);

                if (writeState_ == WriteState::PENDING)
                {
                    outgoingLength = txLength_;
                    outgoingOffset = 0;

                    std::copy_n(
                        tx_.begin(),
                        outgoingLength,
                        outbound.begin());

                    writeState_ = WriteState::IN_PROGRESS;
                }
            }

            // ------------------------------------------------
            // Write available bytes to the physical device.
            // ------------------------------------------------
            if (outgoingLength != 0)
            {
                const std::span<const std::uint8_t> remaining(
                    outbound.data() + outgoingOffset,
                    outgoingLength - outgoingOffset);

                const HilIoResult result =
                    device_->write(remaining);

                if (!result.ok ||
                    result.bytes > remaining.size())
                {
                    fail(result.error);
                    break;
                }

                if (result.bytes != 0)
                {
                    outgoingOffset += result.bytes;
                    transmittedBytes_.fetch_add(result.bytes);
                    madeProgress = true;
                }

                if (outgoingOffset == outgoingLength)
                {
                    std::lock_guard lock(txMutex_);

                    writeState_ = WriteState::COMPLETED;
                    outgoingLength = 0;
                    outgoingOffset = 0;
                }
            }

            // ------------------------------------------------
            // Receive currently available bytes.
            // ------------------------------------------------
            const HilIoResult received =
                device_->read(incoming);

            if (!received.ok ||
                received.bytes > incoming.size())
            {
                fail(received.error);
                break;
            }

            if (received.bytes != 0)
            {
                const std::size_t pushed =
                    pushReceived(
                        std::span<const std::uint8_t>(
                            incoming.data(),
                            received.bytes));

                const std::size_t dropped =
                    received.bytes - pushed;

                if (dropped != 0)
                {
                    droppedReceiveBytes_.fetch_add(dropped);

                    // Fail closed. The parser can no longer
                    // assume a complete serial byte stream.
                    fail(HIL_SERIAL_RX_OVERFLOW);
                    break;
                }

                madeProgress = true;
            }

            if (!madeProgress)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1));
            }
        }

        device_->close();

        if (state_ != HilSerialWorkerState::FAILED)
        {
            state_ = HilSerialWorkerState::STOPPED;
        }
    }
}
