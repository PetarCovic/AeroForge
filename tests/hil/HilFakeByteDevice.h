#pragma once

#include <hil/HilByteDevice.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <span>

namespace aeroforge::hil::tests
{
    struct FakeByteDeviceState
    {
        std::mutex mutex;

        std::deque<std::uint8_t> incoming;
        std::deque<std::uint8_t> outgoing;

        std::atomic<bool> allowOpen{ true };
        std::atomic<bool> failReads{ false };
        std::atomic<bool> failWrites{ false };
        std::atomic<bool> blockWrites{ false };

        std::size_t maxReadChunk = 256;
        std::size_t maxWriteChunk = 256;

        void inject(std::span<const std::uint8_t> bytes)
        {
            std::lock_guard lock(mutex);

            for (std::uint8_t byte : bytes)
                incoming.push_back(byte);
        }

        std::deque<std::uint8_t> takeOutgoing()
        {
            std::lock_guard lock(mutex);

            std::deque<std::uint8_t> result;
            result.swap(outgoing);

            return result;
        }
    };

    class HilFakeByteDevice final : public HilByteDevice
    {
    public:
        explicit HilFakeByteDevice(
            std::shared_ptr<FakeByteDeviceState> state)
            : state_(std::move(state))
        {
        }

        bool open() override
        {
            open_ = state_->allowOpen.load();
            error_ = open_ ? 0u : 1001u;

            return open_;
        }

        void close() noexcept override
        {
            open_ = false;
        }

        HilIoResult read(
            std::span<std::uint8_t> output) override
        {
            if (!open_ || state_->failReads.load())
            {
                error_ = 1002;
                return { false, 0, error_ };
            }

            std::lock_guard lock(state_->mutex);

            const std::size_t count = std::min({
                output.size(),
                state_->incoming.size(),
                state_->maxReadChunk
                });

            for (std::size_t i = 0; i < count; ++i)
            {
                output[i] = state_->incoming.front();
                state_->incoming.pop_front();
            }

            return { true, count, 0 };
        }

        HilIoResult write(
            std::span<const std::uint8_t> input) override
        {
            if (!open_ || state_->failWrites.load())
            {
                error_ = 1003;
                return { false, 0, error_ };
            }

            if (state_->blockWrites.load())
            {
                return { true, 0, 0 };
            }

            std::lock_guard lock(state_->mutex);

            const std::size_t count =
                std::min(input.size(), state_->maxWriteChunk);

            for (std::size_t i = 0; i < count; ++i)
                state_->outgoing.push_back(input[i]);

            return { true, count, 0 };
        }

        std::uint32_t lastError() const noexcept override
        {
            return error_;
        }

    private:
        std::shared_ptr<FakeByteDeviceState> state_;
        bool open_ = false;
        std::uint32_t error_ = 0;
    };
}
