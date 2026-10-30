#include <hil/HilSerializer.h>

#include <cmath>
#include <bit>
#include <limits>
#include <array>
#include <algorithm>

namespace aeroforge::hil
{
	bool readU8(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint8_t& value)
	{
		if (offset >= buffer.size())
		{
			return false;
		}

		value = buffer[offset];

		offset++;

		return true;
	}

	bool writeU8(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint8_t value)
	{
		if (offset >= buffer.size())
		{
			return false;
		}
		
		buffer[offset] = value;

		offset++;

		return true;
	}

	bool readU16(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint16_t& value)
	{
		if (offset > buffer.size() ||
			buffer.size() - offset < 2)
		{
			return false;
		}

		std::uint16_t lowByte =
			static_cast<std::uint16_t>(buffer[offset]);

		std::uint16_t highByte =
			static_cast<std::uint16_t>(buffer[offset + 1]);

		value = lowByte | (highByte << 8);

		offset += 2;

		return true;
	}

	bool writeU16(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint16_t value)
	{
		if (offset > buffer.size() ||
			buffer.size() - offset < 2)
		{
			return false;
		}

		std::uint8_t lowByte = static_cast<std::uint8_t>(value & 0xFF);
		std::uint8_t highByte = static_cast<std::uint8_t>((value >> 8) & 0xFF);

		buffer[offset] = lowByte;
		buffer[offset + 1] = highByte;

		offset+=2;

		return true;
	}

	bool readU32(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint32_t& value)
	{
		if (offset > buffer.size() ||
			buffer.size() - offset < 4)
		{
			return false;
		}

		std::uint32_t lowByte =
			static_cast<std::uint32_t>(buffer[offset]);

		std::uint32_t midLowByte =
			static_cast<std::uint32_t>(buffer[offset + 1]);

		std::uint32_t midHighByte =
			static_cast<std::uint32_t>(buffer[offset + 2]);

		std::uint32_t highByte =
			static_cast<std::uint32_t>(buffer[offset + 3]);

		value = lowByte | (midLowByte << 8) | (midHighByte << 16) | (highByte << 24);

		offset += 4;

		return true;
	}

	bool writeU32(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint32_t value)
	{
		if (offset > buffer.size() ||
			buffer.size() - offset < 4)
		{
			return false;
		}

		std::uint8_t lowByte = static_cast<std::uint8_t>(value & 0xFF);
		std::uint8_t midLowByte = static_cast<std::uint8_t>((value >> 8) & 0xFF);
		std::uint8_t midHighByte = static_cast<std::uint8_t>((value >> 16) & 0xFF);
		std::uint8_t highByte = static_cast<std::uint8_t>((value >> 24) & 0xFF);

		buffer[offset] = lowByte;
		buffer[offset + 1] = midLowByte;
		buffer[offset + 2] = midHighByte;
		buffer[offset + 3] = highByte;

		offset+=4;

		return true;
	}

	bool readU64(std::span<const std::uint8_t> buffer, std::size_t& offset, std::uint64_t& value)
	{
		if (offset > buffer.size() ||
			buffer.size() - offset < 8)
		{
			return false;
		}

		std::uint64_t lowByte1 =
			static_cast<std::uint64_t>(buffer[offset]);
		std::uint64_t midLowByte1 =
			static_cast<std::uint64_t>(buffer[offset + 1]);
		std::uint64_t midHighByte1 =
			static_cast<std::uint64_t>(buffer[offset + 2]);
		std::uint64_t highByte1 =
			static_cast<std::uint64_t>(buffer[offset + 3]);

		std::uint64_t lowByte2 =
			static_cast<std::uint64_t>(buffer[offset + 4]);
		std::uint64_t midLowByte2 =
			static_cast<std::uint64_t>(buffer[offset + 5]);
		std::uint64_t midHighByte2 =
			static_cast<std::uint64_t>(buffer[offset + 6]);
		std::uint64_t highByte2 =
			static_cast<std::uint64_t>(buffer[offset + 7]);

		value = lowByte1 | (midLowByte1 << 8) | (midHighByte1 << 16) | (highByte1 << 24) | (lowByte2 << 32) | (midLowByte2 << 40) | (midHighByte2 << 48) | (highByte2 << 56);

		offset += 8;

		return true;
	}

	bool writeU64(std::span<std::uint8_t> buffer, std::size_t& offset, std::uint64_t value)
	{
		if (offset > buffer.size() ||
			buffer.size() - offset < 8)
		{
			return false;
		}

		std::uint8_t lowByte1 = static_cast<std::uint8_t>(value & 0xFF);
		std::uint8_t midLowByte1 = static_cast<std::uint8_t>((value >> 8) & 0xFF);
		std::uint8_t midHighByte1 = static_cast<std::uint8_t>((value >> 16) & 0xFF);
		std::uint8_t highByte1 = static_cast<std::uint8_t>((value >> 24) & 0xFF);

		std::uint8_t lowByte2 = static_cast<std::uint8_t>((value >> 32) & 0xFF);
		std::uint8_t midLowByte2 = static_cast<std::uint8_t>((value >> 40) & 0xFF);
		std::uint8_t midHighByte2 = static_cast<std::uint8_t>((value >> 48) & 0xFF);
		std::uint8_t highByte2 = static_cast<std::uint8_t>((value >> 56) & 0xFF);

		buffer[offset] = lowByte1;
		buffer[offset + 1] = midLowByte1;
		buffer[offset + 2] = midHighByte1;
		buffer[offset + 3] = highByte1;
		buffer[offset + 4] = lowByte2;
		buffer[offset + 5] = midLowByte2;
		buffer[offset + 6] = midHighByte2;
		buffer[offset + 7] = highByte2;

		offset+=8;

		return true;
	}

	bool readF32(std::span<const std::uint8_t> buffer, std::size_t& offset, float& value)
	{
		static_assert(sizeof(float) == sizeof(std::uint32_t));
		static_assert(std::numeric_limits<float>::is_iec559);

		std::uint32_t tmp = 0;
		std::size_t tmpOffset = offset;

		if (!readU32(buffer, tmpOffset, tmp))
		{
			return false;
		}

		float flt = std::bit_cast<float>(tmp);

		if (!std::isfinite(flt))
		{
			return false;
		}

		value = flt;

		offset = tmpOffset;

		return true;
	}

	bool writeF32(std::span<std::uint8_t> buffer, std::size_t& offset, float value)
	{
		static_assert(sizeof(float) == sizeof(std::uint32_t));
		static_assert(std::numeric_limits<float>::is_iec559);

		if (!std::isfinite(value))
		{
			return false;
		}

		std::uint32_t bits = std::bit_cast<std::uint32_t>(value);

		return writeU32(buffer, offset, bits);
	}

	bool readHeader(std::span<const std::uint8_t> buffer, std::size_t& offset, HilHeader& header)
	{
		if (offset > buffer.size() || buffer.size() - offset < HEADER_SIZE)
		{
			return false;
		}

		// Decode into temporary state to preserve the caller's
		// header and offset if decoding or validation fails.
		HilHeader tmpHeader{};
		std::size_t tmpOffset = offset;

		if (!readU8(buffer, tmpOffset, tmpHeader.version) ||
			!readU8(buffer, tmpOffset, tmpHeader.messageType) ||
			!readU16(buffer, tmpOffset, tmpHeader.payloadLength) ||
			!readU32(buffer, tmpOffset, tmpHeader.sequence))
		{
			return false;
		}

		// Validate protocol version.
		if (tmpHeader.version != PROTOCOL_VERSION)
		{
			return false;
		}

		// Validate message type.
		if (tmpHeader.messageType <
			static_cast<std::uint8_t>(MessageType::HELLO) ||
			tmpHeader.messageType >
			static_cast<std::uint8_t>(MessageType::FAULT))
		{
			return false;
		}

		// Validate the declared payload size.
		if (tmpHeader.payloadLength > MAX_PAYLOAD_SIZE)
		{
			return false;
		}

		// Commit only after successful decoding and validation.
		header = tmpHeader;
		offset = tmpOffset;

		return true;
	}

	bool writeHeader(std::span<std::uint8_t> buffer, std::size_t& offset, const HilHeader& header)
	{
		// Validate protocol version.
		if (header.version != PROTOCOL_VERSION)
		{
			return false;
		}

		// Validate message type.
		if (header.messageType < static_cast<std::uint8_t>(MessageType::HELLO) || header.messageType > static_cast<std::uint8_t>(MessageType::FAULT))
		{
			return false;
		}

		// Validate the declared payload size.
		if (header.payloadLength > MAX_PAYLOAD_SIZE)
		{
			return false;
		}

		// Ensure all eight header bytes fit before writing.
		if (offset > buffer.size() ||
			buffer.size() - offset < HEADER_SIZE)
		{
			return false;
		}

		std::size_t tmpOffset = offset;

		// Encode each field in the protocol-defined order.
		if (!writeU8(buffer, tmpOffset, header.version) 
			|| !writeU8(buffer, tmpOffset, header.messageType) 
			|| !writeU16(buffer, tmpOffset, header.payloadLength) 
			|| !writeU32(buffer, tmpOffset, header.sequence))
		{
			return false;
		}

		// Commit the updated offset after successful encoding.
		offset = tmpOffset;

		return true;
	}

	bool readSensorSample(std::span<const std::uint8_t> buffer, std::size_t& offset, SensorSampleMessage& sample)
	{
		SensorSampleMessage tmpSample{};
		std::size_t tmpOffset = offset;

		if (!readU64(buffer, tmpOffset, tmpSample.simulationTimeUs)
			|| !readVector3(buffer, tmpOffset, tmpSample.specificForceB)
			|| !readVector3(buffer, tmpOffset, tmpSample.angularVelocityB))
		{
			return false;
		}

		sample = tmpSample;

		offset = tmpOffset;

		return true;
	}

	bool writeSensorSample(std::span<std::uint8_t> buffer, std::size_t& offset, const SensorSampleMessage& sample)
	{
		if (offset > buffer.size() || buffer.size() - offset < SENSOR_SAMPLE_PAYLOAD_SIZE)
		{
			return false;
		}

		std::array<std::uint8_t, SENSOR_SAMPLE_PAYLOAD_SIZE> tmpBuffer{};
		std::size_t tmpOffset = 0;

		if (!writeU64(tmpBuffer, tmpOffset, sample.simulationTimeUs)
			|| !writeVector3(tmpBuffer, tmpOffset, sample.specificForceB)
			|| !writeVector3(tmpBuffer, tmpOffset, sample.angularVelocityB))
		{
			return false;
		}

		std::copy(
			tmpBuffer.begin(),
			tmpBuffer.end(),
			buffer.begin() + offset
		);

		offset += SENSOR_SAMPLE_PAYLOAD_SIZE;

		return true;
	}

	bool readControlCommand(std::span<const std::uint8_t> buffer, std::size_t& offset, ControlCommandMessage& command)
	{
		ControlCommandMessage tmpCommand{};
		std::size_t tmpOffset = offset;

		if (!readU32(buffer, tmpOffset, tmpCommand.referencedSensorSequence)
			|| !readVector3(buffer, tmpOffset, tmpCommand.commandedMomentB)
			|| !readU8(buffer, tmpOffset, tmpCommand.controllerStatus))
		{
			return false;
		}

		command = tmpCommand;

		offset = tmpOffset;

		return true;
	}

	bool writeControlCommand(std::span<std::uint8_t> buffer, std::size_t& offset, const ControlCommandMessage& command)
	{
		if (offset > buffer.size() || buffer.size() - offset < CONTROL_COMMAND_PAYLOAD_SIZE)
		{
			return false;
		}

		std::array<std::uint8_t, CONTROL_COMMAND_PAYLOAD_SIZE> tmpBuffer{};
		std::size_t tmpOffset = 0;

		if (!writeU32(tmpBuffer, tmpOffset, command.referencedSensorSequence)
			|| !writeVector3(tmpBuffer, tmpOffset, command.commandedMomentB)
			|| !writeU8(tmpBuffer, tmpOffset, command.controllerStatus))
		{
			return false;
		}

		std::copy(
			tmpBuffer.begin(),
			tmpBuffer.end(),
			buffer.begin() + offset
		);

		offset += CONTROL_COMMAND_PAYLOAD_SIZE;

		return true;
	}

	bool readVector3(std::span<const std::uint8_t> buffer, std::size_t& offset, math::Vector3& value)
	{
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;

		std::size_t tmpOffset = offset;

		if (!readF32(buffer, tmpOffset, x) 
			|| !readF32(buffer, tmpOffset, y) 
			|| !readF32(buffer, tmpOffset, z))
		{
			return false;
		}

		value = math::Vector3(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));

		offset = tmpOffset;

		return true;
	}

	bool writeVector3(std::span<std::uint8_t> buffer, std::size_t& offset, const math::Vector3& value)
	{
		constexpr double maxF32 = static_cast<double>(std::numeric_limits<float>::max());

		if (!std::isfinite(value.x) || value.x < -maxF32 || value.x > maxF32)
		{
			return false;
		}

		if (!std::isfinite(value.y) || value.y < -maxF32 || value.y > maxF32)
		{
			return false;
		}

		if (!std::isfinite(value.z) || value.z < -maxF32 || value.z > maxF32)
		{
			return false;
		}

		if (offset > buffer.size() || buffer.size() - offset < VECTOR3_WIRE_SIZE)
		{
			return false;
		}

		std::size_t tmpOffset = offset;

		if (!writeF32(buffer, tmpOffset, static_cast<float>(value.x))
			|| !writeF32(buffer, tmpOffset, static_cast<float>(value.y))
			|| !writeF32(buffer, tmpOffset, static_cast<float>(value.z)))
		{
			return false;
		}

		offset = tmpOffset;

		return true;
	}
}