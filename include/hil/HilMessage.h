#pragma once

#include <math/Vector3.h>

#include <cstdint>
#include <cstddef>

namespace aeroforge::hil
{
	inline constexpr std::uint8_t PROTOCOL_VERSION = 1;
	inline constexpr std::size_t MAX_RAW_PACKET_SIZE = 128;
	inline constexpr std::size_t HEADER_SIZE = 8;
	inline constexpr std::size_t CRC_SIZE = 4;
	inline constexpr std::size_t MAX_PAYLOAD_SIZE = MAX_RAW_PACKET_SIZE - HEADER_SIZE - CRC_SIZE;
	inline constexpr std::size_t SENSOR_SAMPLE_PAYLOAD_SIZE = 32;
	inline constexpr std::size_t CONTROL_COMMAND_PAYLOAD_SIZE = 17;
	inline constexpr std::size_t VECTOR3_WIRE_SIZE = 12;
	inline constexpr std::size_t MAX_COBS_ENCODED_SIZE = 129;
	inline constexpr std::size_t MAX_FRAMED_PACKET_SIZE = 130;
	inline constexpr std::size_t SENSOR_PACKET_SIZE = HEADER_SIZE + SENSOR_SAMPLE_PAYLOAD_SIZE + CRC_SIZE;
	inline constexpr std::size_t CONTROL_PACKET_SIZE = HEADER_SIZE + CONTROL_COMMAND_PAYLOAD_SIZE + CRC_SIZE;

	enum class MessageType : std::uint8_t
	{
		HELLO = 1,
		SENSOR_SAMPLE = 2,
		CONTROL_COMMAND = 3,
		STATUS = 4,
		FAULT = 5
	};

	struct HilHeader
	{
		std::uint8_t version = PROTOCOL_VERSION;
		std::uint8_t messageType = 0;
		std::uint16_t payloadLength = 0;
		std::uint32_t sequence = 0;
	};

	struct SensorSampleMessage
	{
		std::uint64_t simulationTimeUs = 0;
		math::Vector3 specificForceB;
		math::Vector3 angularVelocityB;
	};

	struct ControlCommandMessage
	{
		std::uint32_t referencedSensorSequence = 0;
		math::Vector3 commandedMomentB;
		std::uint8_t controllerStatus = 0;
	};

	enum class ControllerStatus : std::uint8_t
	{
		OK = 0,
		DEGRADED = 1,
		FAULT = 2
	};
}