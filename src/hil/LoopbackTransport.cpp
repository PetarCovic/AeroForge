#include <hil/LoopbackTransport.h>

#include <stdexcept>
#include <utility>
#include <algorithm>

namespace aeroforge::hil
{
	LoopbackTransport::LoopbackTransport(std::shared_ptr<SharedState> statePtr, Endpoint endpoint)
		: endpoint(endpoint), statePtr(std::move(statePtr))
	{
		if (!this->statePtr)
		{
			throw std::invalid_argument("statePtr cannot be nullptr");
		}
	}

	LoopbackTransport::~LoopbackTransport()
	{
		close();
	}

	std::pair<
		std::unique_ptr<LoopbackTransport>, 
		std::unique_ptr<LoopbackTransport>> 
		LoopbackTransport::createPair()
	{
		auto state = std::make_shared<SharedState>();

		std::unique_ptr<LoopbackTransport> a(new LoopbackTransport(state, Endpoint::A));
		std::unique_ptr<LoopbackTransport> b(new LoopbackTransport(std::move(state), Endpoint::B));
		
		return std::make_pair(std::move(a), std::move(b));
	}

	bool LoopbackTransport::isOpen() const
	{
		std::lock_guard<std::mutex> lock(statePtr->mtx);

		if (endpoint == Endpoint::A)
		{
			return statePtr->endpointAOpen;
		}

		return statePtr->endpointBOpen;
	}

	std::size_t LoopbackTransport::tryRead(std::span<std::uint8_t> buffer)
	{
		std::lock_guard<std::mutex> lock(statePtr->mtx);

		if ((endpoint==Endpoint::A && !statePtr->endpointAOpen) || (endpoint == Endpoint::B && !statePtr->endpointBOpen))
		{
			return 0;
		}

		auto& queue = (endpoint == Endpoint::A)
			? statePtr->bToA
			: statePtr->aToB;

		std::size_t bytesRead = std::min(buffer.size(), queue.size());

		for (std::size_t i = 0; i < bytesRead; i++)
		{
			buffer[i] = queue.front();
			queue.pop_front();
		}

		return bytesRead;
	}

	std::size_t LoopbackTransport::tryWrite(std::span <const std::uint8_t> data)
	{
		std::lock_guard<std::mutex> lock(statePtr->mtx);

		if (!statePtr->endpointAOpen || !statePtr->endpointBOpen)
		{
			return 0;
		}

		auto& queue = (endpoint == Endpoint::A)
			? statePtr->aToB
			: statePtr->bToA;

		std::size_t bytesAccepted = std::min(data.size(), MAX_CAPACITY - queue.size());
		
		for (std::size_t i = 0; i < bytesAccepted; i++)
		{
			queue.push_back(data[i]);
		}

		return bytesAccepted;
	}

	void LoopbackTransport::close()
	{
		std::lock_guard<std::mutex> lock(statePtr->mtx);

		if (endpoint == Endpoint::A)
		{
			statePtr->endpointAOpen=false;
		}
		else
		{
			statePtr->endpointBOpen=false;
		}
	}

	Endpoint LoopbackTransport::getEndpoint() const
	{
		return endpoint;
	}
}