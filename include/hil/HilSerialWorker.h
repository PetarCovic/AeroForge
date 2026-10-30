#pragma once

#include <hil/HilByteDevice.h>
#include <hil/HilWireEndpoint.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <thread>

namespace aeroforge::hil
{
    inline constexpr std::uint32_t HIL_SERIAL_RX_OVERFLOW = 0xE0010001u;

    enum class HilSerialWorkerState
    {
        STOPPED,
        STARTING,
        RUNNING,
        FAILED
    };

    class HilSerialWorker
    {
    public:
        static constexpr std::size_t RX_CAPACITY = 8192;

        explicit HilSerialWorker(
            std::unique_ptr<HilByteDevice> device);

        ~HilSerialWorker();

        HilSerialWorker(const HilSerialWorker&) = delete;
        HilSerialWorker& operator=(
            const HilSerialWorker&) = delete;

        // Starts the worker asynchronously.
        // true means thread creation succeeded,
        // not that the COM port has opened.
        bool start();

        // Requests shutdown and joins the worker.
        void stop();

        // Nonblocking byte port for HilWireEndpoint.
        HilBytePort makePort();

        HilSerialWorkerState state() const noexcept;

        std::uint32_t lastError() const noexcept;

        std::uint64_t receivedBytes() const noexcept;
        std::uint64_t transmittedBytes() const noexcept;
        std::uint64_t droppedReceiveBytes() const noexcept;

    private:
        std::unique_ptr<HilByteDevice> device_;

        std::jthread thread_;

        std::atomic<HilSerialWorkerState> state_{
            HilSerialWorkerState::STOPPED
        };

        std::atomic<std::uint32_t> lastError_{ 0 };

        std::atomic<std::uint64_t> receivedBytes_{ 0 };
        std::atomic<std::uint64_t> transmittedBytes_{ 0 };
        std::atomic<std::uint64_t> droppedReceiveBytes_{ 0 };

        // Thread-safe bounded receive buffer.
        std::mutex rxMutex_;

        std::array<std::uint8_t, RX_CAPACITY> rx_{};

        std::size_t rxHead_ = 0;
        std::size_t rxTail_ = 0;
        std::size_t rxSize_ = 0;

        // One outstanding, bounded write request.
        enum class WriteState
        {
            IDLE,
            PENDING,
            IN_PROGRESS,
            COMPLETED
        };

        std::mutex txMutex_;

        std::array<std::uint8_t,
            MAX_FRAMED_PACKET_SIZE> tx_{};

        std::size_t txLength_ = 0;
        WriteState writeState_ = WriteState::IDLE;

        std::size_t popReceived(
            std::span<std::uint8_t> output);

        std::size_t pushReceived(
            std::span<const std::uint8_t> input);

        std::size_t requestWrite(
            std::span<const std::uint8_t> input);

        void run(std::stop_token stopToken);

        void fail(std::uint32_t error);
    };
}
