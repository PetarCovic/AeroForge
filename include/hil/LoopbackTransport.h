#pragma once

#include <hil/HilTransport.h>

#include <mutex>
#include <deque>
#include <memory>
#include <utility>

namespace aeroforge::hil
{
	struct SharedState
	{
		std::mutex mtx;
		std::deque<std::uint8_t> aToB;
		std::deque<std::uint8_t> bToA;
		bool endpointAOpen=true;
		bool endpointBOpen=true;
	};

	enum class Endpoint
	{
		A,
		B
	};

	class LoopbackTransport : public HilTransport
	{
	public:
		static constexpr std::size_t MAX_CAPACITY = 4096;

		~LoopbackTransport() override;

		LoopbackTransport(const LoopbackTransport&) = delete;
		LoopbackTransport& operator=(const LoopbackTransport&) = delete;

		LoopbackTransport(LoopbackTransport&&) = delete;
		LoopbackTransport& operator=(LoopbackTransport&&) = delete;

		static std::pair<std::unique_ptr<LoopbackTransport>, std::unique_ptr<LoopbackTransport>> createPair();

		bool isOpen() const override;

		std::size_t tryRead(std::span<std::uint8_t> buffer) override;

		std::size_t tryWrite(std::span <const std::uint8_t> data) override;

		void close() override;

		Endpoint getEndpoint() const;

	private:
		LoopbackTransport(std::shared_ptr<SharedState> statePtr, Endpoint endpoint);

		Endpoint endpoint;
		std::shared_ptr<SharedState> statePtr;
	};
}