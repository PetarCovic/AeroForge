#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace aeroforge::hil
{
	class HilTransport
	{
	public:
		virtual ~HilTransport() = default;

		virtual bool isOpen() const = 0;

		virtual std::size_t tryRead(std::span<std::uint8_t> buffer) = 0;

		virtual std::size_t tryWrite(std::span <const std::uint8_t> data) = 0;

		virtual void close() = 0;
	};
}