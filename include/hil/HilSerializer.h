#pragma once

#include <hil/HilMessage.h>

#include <cstdint>
#include <cstddef>
#include <span>

namespace aeroforge::hil
{
	bool readU8(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint8_t& value);

	bool writeU8(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint8_t value);

	bool readU16(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint16_t& value);

	bool writeU16(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint16_t value);

	bool readU32(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint32_t& value);

	bool writeU32(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint32_t value);

	bool readU64(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint64_t& value);

	bool writeU64(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint64_t value);

	bool readF32(std::span<const std::uint8_t> buffer, std::size_t& offset, float& value);

	bool writeF32(std::span<std::uint8_t> buffer, std::size_t& offset, float value);

	bool readHeader(std::span<const std::uint8_t> buffer, std::size_t& offset, HilHeader& header);

	bool writeHeader(std::span<std::uint8_t> buffer, std::size_t& offset, const HilHeader& header);

	bool readSensorSample(std::span<const std::uint8_t> buffer, std::size_t& offset, SensorSampleMessage& sample);

	bool writeSensorSample(std::span<std::uint8_t> buffer, std::size_t& offset, const SensorSampleMessage& sample);

	bool readControlCommand(std::span<const std::uint8_t> buffer, std::size_t& offset, ControlCommandMessage& command);

	bool writeControlCommand(std::span<std::uint8_t> buffer, std::size_t& offset, const ControlCommandMessage& command);

	bool readVector3(std::span<const std::uint8_t> buffer, std::size_t& offset, math::Vector3& value);
	
	bool writeVector3(std::span<std::uint8_t> buffer, std::size_t& offset, const math::Vector3& value);
}