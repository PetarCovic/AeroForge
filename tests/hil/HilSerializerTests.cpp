
#include <gtest/gtest.h>
#include <hil/HilSerializer.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <bit>
#include <cmath>

using namespace aeroforge::hil;

// ============================================================
// 1. BASIC 8-BIT TESTS
// ============================================================

TEST(HilSerializerTests, WriteU8)
{
    std::array<std::uint8_t, 1> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeU8(buffer, offset, 0xAB));

    EXPECT_EQ(buffer[0], 0xAB);
    EXPECT_EQ(offset, 1u);
}

TEST(HilSerializerTests, ReadU8)
{
    const std::array<std::uint8_t, 1> buffer{0xAB};
    std::size_t offset = 0;
    std::uint8_t value = 0;

    ASSERT_TRUE(readU8(buffer, offset, value));

    EXPECT_EQ(value, 0xAB);
    EXPECT_EQ(offset, 1u);
}

// ============================================================
// 2. BASIC 16-BIT TESTS
// ============================================================

TEST(HilSerializerTests, WriteU16LittleEndian)
{
    std::array<std::uint8_t, 2> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeU16(buffer, offset, 0xABCD));

    EXPECT_EQ(buffer[0], 0xCD);
    EXPECT_EQ(buffer[1], 0xAB);
    EXPECT_EQ(offset, 2u);
}

TEST(HilSerializerTests, ReadU16LittleEndian)
{
    const std::array<std::uint8_t, 2> buffer{
        0xCD, 0xAB
    };

    std::size_t offset = 0;
    std::uint16_t value = 0;

    ASSERT_TRUE(readU16(buffer, offset, value));

    EXPECT_EQ(value, 0xABCD);
    EXPECT_EQ(offset, 2u);
}

// ============================================================
// 3. BASIC 32-BIT TESTS
// ============================================================

TEST(HilSerializerTests, WriteU32LittleEndian)
{
    std::array<std::uint8_t, 4> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeU32(buffer, offset, 0x12345678u));

    EXPECT_EQ(buffer[0], 0x78);
    EXPECT_EQ(buffer[1], 0x56);
    EXPECT_EQ(buffer[2], 0x34);
    EXPECT_EQ(buffer[3], 0x12);
    EXPECT_EQ(offset, 4u);
}

TEST(HilSerializerTests, ReadU32LittleEndian)
{
    const std::array<std::uint8_t, 4> buffer{
        0x78, 0x56, 0x34, 0x12
    };

    std::size_t offset = 0;
    std::uint32_t value = 0;

    ASSERT_TRUE(readU32(buffer, offset, value));

    EXPECT_EQ(value, 0x12345678u);
    EXPECT_EQ(offset, 4u);
}

// ============================================================
// 4. BASIC 64-BIT TESTS
// ============================================================

TEST(HilSerializerTests, WriteU64LittleEndian)
{
    std::array<std::uint8_t, 8> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeU64(
        buffer, offset, 0x1122334455667788ULL
    ));

    const std::array<std::uint8_t, 8> expected{
        0x88, 0x77, 0x66, 0x55,
        0x44, 0x33, 0x22, 0x11
    };

    EXPECT_EQ(buffer, expected);
    EXPECT_EQ(offset, 8u);
}

TEST(HilSerializerTests, ReadU64LittleEndian)
{
    const std::array<std::uint8_t, 8> buffer{
        0x88, 0x77, 0x66, 0x55,
        0x44, 0x33, 0x22, 0x11
    };

    std::size_t offset = 0;
    std::uint64_t value = 0;

    ASSERT_TRUE(readU64(buffer, offset, value));

    EXPECT_EQ(value, 0x1122334455667788ULL);
    EXPECT_EQ(offset, 8u);
}

// ============================================================
// 5. SEQUENTIAL SERIALIZATION
// ============================================================

TEST(HilSerializerTests, SequentialWrites)
{
    std::array<std::uint8_t, 15> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeU8(buffer, offset, 0xAB));
    EXPECT_EQ(offset, 1u);

    ASSERT_TRUE(writeU16(buffer, offset, 0x1234));
    EXPECT_EQ(offset, 3u);

    ASSERT_TRUE(writeU32(buffer, offset, 0x12345678u));
    EXPECT_EQ(offset, 7u);

    ASSERT_TRUE(writeU64(
        buffer, offset, 0x1122334455667788ULL
    ));
    EXPECT_EQ(offset, 15u);

    const std::array<std::uint8_t, 15> expected{
        0xAB,
        0x34, 0x12,
        0x78, 0x56, 0x34, 0x12,
        0x88, 0x77, 0x66, 0x55,
        0x44, 0x33, 0x22, 0x11
    };

    EXPECT_EQ(buffer, expected);
}

TEST(HilSerializerTests, SequentialReads)
{
    const std::array<std::uint8_t, 15> buffer{
        0xAB,
        0x34, 0x12,
        0x78, 0x56, 0x34, 0x12,
        0x88, 0x77, 0x66, 0x55,
        0x44, 0x33, 0x22, 0x11
    };

    std::size_t offset = 0;

    std::uint8_t value8 = 0;
    std::uint16_t value16 = 0;
    std::uint32_t value32 = 0;
    std::uint64_t value64 = 0;

    ASSERT_TRUE(readU8(buffer, offset, value8));
    ASSERT_TRUE(readU16(buffer, offset, value16));
    ASSERT_TRUE(readU32(buffer, offset, value32));
    ASSERT_TRUE(readU64(buffer, offset, value64));

    EXPECT_EQ(value8, 0xAB);
    EXPECT_EQ(value16, 0x1234);
    EXPECT_EQ(value32, 0x12345678u);
    EXPECT_EQ(value64, 0x1122334455667788ULL);

    EXPECT_EQ(offset, buffer.size());
}

// ============================================================
// 6. ROUND-TRIP SERIALIZATION
// ============================================================

TEST(HilSerializerTests, RoundTripAllIntegerTypes)
{
    std::array<std::uint8_t, 15> buffer{};

    std::size_t writeOffset = 0;

    ASSERT_TRUE(writeU8(buffer, writeOffset, 0xFE));
    ASSERT_TRUE(writeU16(buffer, writeOffset, 0xFEDC));
    ASSERT_TRUE(writeU32(buffer, writeOffset, 0xFEDCBA98u));
    ASSERT_TRUE(writeU64(
        buffer, writeOffset, 0xFEDCBA9876543210ULL
    ));

    EXPECT_EQ(writeOffset, buffer.size());

    std::size_t readOffset = 0;

    std::uint8_t value8 = 0;
    std::uint16_t value16 = 0;
    std::uint32_t value32 = 0;
    std::uint64_t value64 = 0;

    ASSERT_TRUE(readU8(buffer, readOffset, value8));
    ASSERT_TRUE(readU16(buffer, readOffset, value16));
    ASSERT_TRUE(readU32(buffer, readOffset, value32));
    ASSERT_TRUE(readU64(buffer, readOffset, value64));

    EXPECT_EQ(value8, 0xFE);
    EXPECT_EQ(value16, 0xFEDC);
    EXPECT_EQ(value32, 0xFEDCBA98u);
    EXPECT_EQ(value64, 0xFEDCBA9876543210ULL);

    EXPECT_EQ(readOffset, writeOffset);
}

// ============================================================
// 7. NONZERO STARTING OFFSET
// ============================================================

TEST(HilSerializerTests, WriteWithNonzeroOffset)
{
    std::array<std::uint8_t, 6> buffer{};
    buffer.fill(0xAA);

    std::size_t offset = 2;

    ASSERT_TRUE(writeU16(buffer, offset, 0x1234));

    const std::array<std::uint8_t, 6> expected{
        0xAA, 0xAA, 0x34, 0x12, 0xAA, 0xAA
    };

    EXPECT_EQ(buffer, expected);
    EXPECT_EQ(offset, 4u);
}

TEST(HilSerializerTests, ReadWithNonzeroOffset)
{
    const std::array<std::uint8_t, 6> buffer{
        0xAA, 0xAA, 0x34, 0x12, 0xBB, 0xBB
    };

    std::size_t offset = 2;
    std::uint16_t value = 0;

    ASSERT_TRUE(readU16(buffer, offset, value));

    EXPECT_EQ(value, 0x1234);
    EXPECT_EQ(offset, 4u);
}

// ============================================================
// 8. BOUNDARY VALUES
// ============================================================

TEST(HilSerializerTests, ZeroValues)
{
    std::array<std::uint8_t, 15> buffer{};
    buffer.fill(0xAA);

    std::size_t offset = 0;

    ASSERT_TRUE(writeU8(buffer, offset, 0));
    ASSERT_TRUE(writeU16(buffer, offset, 0));
    ASSERT_TRUE(writeU32(buffer, offset, 0));
    ASSERT_TRUE(writeU64(buffer, offset, 0));

    EXPECT_EQ(offset, 15u);

    for (std::uint8_t byte : buffer)
    {
        EXPECT_EQ(byte, 0);
    }

    std::size_t readOffset = 0;

    std::uint8_t value8 = 1;
    std::uint16_t value16 = 1;
    std::uint32_t value32 = 1;
    std::uint64_t value64 = 1;

    ASSERT_TRUE(readU8(buffer, readOffset, value8));
    ASSERT_TRUE(readU16(buffer, readOffset, value16));
    ASSERT_TRUE(readU32(buffer, readOffset, value32));
    ASSERT_TRUE(readU64(buffer, readOffset, value64));

    EXPECT_EQ(value8, 0);
    EXPECT_EQ(value16, 0);
    EXPECT_EQ(value32, 0);
    EXPECT_EQ(value64, 0);
    EXPECT_EQ(readOffset, 15u);
}

TEST(HilSerializerTests, MaximumUnsignedValues)
{
    std::array<std::uint8_t, 15> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeU8(
        buffer, offset,
        std::numeric_limits<std::uint8_t>::max()
    ));

    ASSERT_TRUE(writeU16(
        buffer, offset,
        std::numeric_limits<std::uint16_t>::max()
    ));

    ASSERT_TRUE(writeU32(
        buffer, offset,
        std::numeric_limits<std::uint32_t>::max()
    ));

    ASSERT_TRUE(writeU64(
        buffer, offset,
        std::numeric_limits<std::uint64_t>::max()
    ));

    EXPECT_EQ(offset, 15u);

    for (std::uint8_t byte : buffer)
    {
        EXPECT_EQ(byte, 0xFF);
    }

    std::size_t readOffset = 0;

    std::uint8_t value8 = 0;
    std::uint16_t value16 = 0;
    std::uint32_t value32 = 0;
    std::uint64_t value64 = 0;

    ASSERT_TRUE(readU8(buffer, readOffset, value8));
    ASSERT_TRUE(readU16(buffer, readOffset, value16));
    ASSERT_TRUE(readU32(buffer, readOffset, value32));
    ASSERT_TRUE(readU64(buffer, readOffset, value64));

    EXPECT_EQ(value8, std::numeric_limits<std::uint8_t>::max());
    EXPECT_EQ(value16, std::numeric_limits<std::uint16_t>::max());
    EXPECT_EQ(value32, std::numeric_limits<std::uint32_t>::max());
    EXPECT_EQ(value64, std::numeric_limits<std::uint64_t>::max());

    EXPECT_EQ(readOffset, 15u);
}

// ============================================================
// 9. INSUFFICIENT WRITE CAPACITY
// ============================================================

TEST(HilSerializerTests, InsufficientWriteCapacity)
{
    std::array<std::uint8_t, 0> buffer8{};
    std::array<std::uint8_t, 1> buffer16{0xAA};
    std::array<std::uint8_t, 3> buffer32{};
    std::array<std::uint8_t, 7> buffer64{};

    buffer32.fill(0xAA);
    buffer64.fill(0xAA);

    std::size_t offset = 0;

    EXPECT_FALSE(writeU8(buffer8, offset, 0x12));
    EXPECT_EQ(offset, 0u);

    EXPECT_FALSE(writeU16(buffer16, offset, 0x1234));
    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer16[0], 0xAA);

    EXPECT_FALSE(writeU32(buffer32, offset, 0x12345678u));
    EXPECT_EQ(offset, 0u);

    for (std::uint8_t byte : buffer32)
    {
        EXPECT_EQ(byte, 0xAA);
    }

    EXPECT_FALSE(writeU64(
        buffer64, offset, 0x1122334455667788ULL
    ));
    EXPECT_EQ(offset, 0u);

    for (std::uint8_t byte : buffer64)
    {
        EXPECT_EQ(byte, 0xAA);
    }
}

// ============================================================
// 10. INSUFFICIENT READ CAPACITY
// ============================================================

TEST(HilSerializerTests, InsufficientReadCapacity)
{
    const std::array<std::uint8_t, 0> buffer8{};
    const std::array<std::uint8_t, 1> buffer16{0xAA};
    const std::array<std::uint8_t, 3> buffer32{
        0xAA, 0xBB, 0xCC
    };
    const std::array<std::uint8_t, 7> buffer64{
        1, 2, 3, 4, 5, 6, 7
    };

    std::size_t offset = 0;

    std::uint8_t value8 = 0x12;
    std::uint16_t value16 = 0x1234;
    std::uint32_t value32 = 0x12345678u;
    std::uint64_t value64 = 0x1122334455667788ULL;

    EXPECT_FALSE(readU8(buffer8, offset, value8));
    EXPECT_EQ(value8, 0x12);
    EXPECT_EQ(offset, 0u);

    EXPECT_FALSE(readU16(buffer16, offset, value16));
    EXPECT_EQ(value16, 0x1234);
    EXPECT_EQ(offset, 0u);

    EXPECT_FALSE(readU32(buffer32, offset, value32));
    EXPECT_EQ(value32, 0x12345678u);
    EXPECT_EQ(offset, 0u);

    EXPECT_FALSE(readU64(buffer64, offset, value64));
    EXPECT_EQ(value64, 0x1122334455667788ULL);
    EXPECT_EQ(offset, 0u);
}

// ============================================================
// 11. OFFSET AT BUFFER END
// ============================================================

TEST(HilSerializerTests, OffsetAtBufferEnd)
{
    std::array<std::uint8_t, 8> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = buffer.size();

    EXPECT_FALSE(writeU8(buffer, offset, 1));
    EXPECT_FALSE(writeU16(buffer, offset, 1));
    EXPECT_FALSE(writeU32(buffer, offset, 1));
    EXPECT_FALSE(writeU64(buffer, offset, 1));

    EXPECT_EQ(offset, 8u);
    EXPECT_EQ(buffer, original);

    std::uint8_t value8 = 11;
    std::uint16_t value16 = 22;
    std::uint32_t value32 = 33;
    std::uint64_t value64 = 44;

    EXPECT_FALSE(readU8(buffer, offset, value8));
    EXPECT_FALSE(readU16(buffer, offset, value16));
    EXPECT_FALSE(readU32(buffer, offset, value32));
    EXPECT_FALSE(readU64(buffer, offset, value64));

    EXPECT_EQ(offset, 8u);
    EXPECT_EQ(value8, 11);
    EXPECT_EQ(value16, 22);
    EXPECT_EQ(value32, 33);
    EXPECT_EQ(value64, 44);
}

// ============================================================
// 12. OFFSET BEYOND BUFFER END
// ============================================================

TEST(HilSerializerTests, OffsetBeyondBufferEnd)
{
    std::array<std::uint8_t, 8> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = 100;

    EXPECT_FALSE(writeU8(buffer, offset, 1));
    EXPECT_FALSE(writeU16(buffer, offset, 1));
    EXPECT_FALSE(writeU32(buffer, offset, 1));
    EXPECT_FALSE(writeU64(buffer, offset, 1));

    EXPECT_EQ(offset, 100u);
    EXPECT_EQ(buffer, original);

    std::uint8_t value8 = 11;
    std::uint16_t value16 = 22;
    std::uint32_t value32 = 33;
    std::uint64_t value64 = 44;

    EXPECT_FALSE(readU8(buffer, offset, value8));
    EXPECT_FALSE(readU16(buffer, offset, value16));
    EXPECT_FALSE(readU32(buffer, offset, value32));
    EXPECT_FALSE(readU64(buffer, offset, value64));

    EXPECT_EQ(offset, 100u);
    EXPECT_EQ(value8, 11);
    EXPECT_EQ(value16, 22);
    EXPECT_EQ(value32, 33);
    EXPECT_EQ(value64, 44);
}

// ============================================================
// 13. NO PARTIAL WRITES NEAR BUFFER END
// ============================================================

TEST(HilSerializerTests, FailedWritesPreserveEntireBuffer)
{
    std::array<std::uint8_t, 8> buffer{};
    buffer.fill(0xA5);

    const auto original = buffer;

    std::size_t offset = 7;

    EXPECT_FALSE(writeU16(buffer, offset, 0x1234));
    EXPECT_EQ(buffer, original);
    EXPECT_EQ(offset, 7u);

    offset = 5;

    EXPECT_FALSE(writeU32(buffer, offset, 0x12345678u));
    EXPECT_EQ(buffer, original);
    EXPECT_EQ(offset, 5u);

    offset = 1;

    EXPECT_FALSE(writeU64(
        buffer, offset, 0x1122334455667788ULL
    ));
    EXPECT_EQ(buffer, original);
    EXPECT_EQ(offset, 1u);
}

// ============================================================
// 14. FAILED READS PRESERVE OUTPUT VALUES
// ============================================================

TEST(HilSerializerTests, FailedReadsPreserveOutputValues)
{
    const std::array<std::uint8_t, 8> buffer{
        1, 2, 3, 4, 5, 6, 7, 8
    };

    std::uint16_t value16 = 0xABCD;
    std::size_t offset = 7;

    EXPECT_FALSE(readU16(buffer, offset, value16));
    EXPECT_EQ(value16, 0xABCD);
    EXPECT_EQ(offset, 7u);

    std::uint32_t value32 = 0x12345678u;
    offset = 5;

    EXPECT_FALSE(readU32(buffer, offset, value32));
    EXPECT_EQ(value32, 0x12345678u);
    EXPECT_EQ(offset, 5u);

    std::uint64_t value64 = 0x1122334455667788ULL;
    offset = 1;

    EXPECT_FALSE(readU64(buffer, offset, value64));
    EXPECT_EQ(value64, 0x1122334455667788ULL);
    EXPECT_EQ(offset, 1u);
}

// ============================================================
// 15. SUCCESSFUL WRITE DOES NOT TOUCH ADJACENT BYTES
// ============================================================

TEST(HilSerializerTests, SuccessfulWritePreservesAdjacentBytes)
{
    std::array<std::uint8_t, 12> buffer{};
    buffer.fill(0xA5);

    std::size_t offset = 4;

    ASSERT_TRUE(writeU32(buffer, offset, 0x12345678u));

    const std::array<std::uint8_t, 12> expected{
        0xA5, 0xA5, 0xA5, 0xA5,
        0x78, 0x56, 0x34, 0x12,
        0xA5, 0xA5, 0xA5, 0xA5
    };

    EXPECT_EQ(buffer, expected);
    EXPECT_EQ(offset, 8u);
}

// ============================================================
// 16. ALL INTEGER TYPES AT EXACT CAPACITY
// ============================================================

TEST(HilSerializerTests, ExactCapacityWrites)
{
    std::array<std::uint8_t, 1> buffer8{};
    std::array<std::uint8_t, 2> buffer16{};
    std::array<std::uint8_t, 4> buffer32{};
    std::array<std::uint8_t, 8> buffer64{};

    std::size_t offset = 0;

    EXPECT_TRUE(writeU8(buffer8, offset, 0xAA));
    EXPECT_EQ(offset, 1u);

    offset = 0;
    EXPECT_TRUE(writeU16(buffer16, offset, 0xABCD));
    EXPECT_EQ(offset, 2u);

    offset = 0;
    EXPECT_TRUE(writeU32(buffer32, offset, 0x12345678u));
    EXPECT_EQ(offset, 4u);

    offset = 0;
    EXPECT_TRUE(writeU64(
        buffer64, offset, 0x1122334455667788ULL
    ));
    EXPECT_EQ(offset, 8u);
}

// ============================================================
// 17. ZERO-LENGTH BUFFERS
// ============================================================

TEST(HilSerializerTests, EmptyBuffersRejectAllOperations)
{
    std::array<std::uint8_t, 0> buffer{};

    std::size_t offset = 0;

    std::uint8_t value8 = 1;
    std::uint16_t value16 = 2;
    std::uint32_t value32 = 3;
    std::uint64_t value64 = 4;

    EXPECT_FALSE(writeU8(buffer, offset, 1));
    EXPECT_FALSE(writeU16(buffer, offset, 1));
    EXPECT_FALSE(writeU32(buffer, offset, 1));
    EXPECT_FALSE(writeU64(buffer, offset, 1));

    EXPECT_FALSE(readU8(buffer, offset, value8));
    EXPECT_FALSE(readU16(buffer, offset, value16));
    EXPECT_FALSE(readU32(buffer, offset, value32));
    EXPECT_FALSE(readU64(buffer, offset, value64));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(value8, 1);
    EXPECT_EQ(value16, 2);
    EXPECT_EQ(value32, 3);
    EXPECT_EQ(value64, 4);
}


// ============================================================
// FLOATING-POINT SERIALIZATION TESTS
// ============================================================

TEST(HilSerializerTests, WriteF32Positive)
{
    std::array<std::uint8_t, 4> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeF32(buffer, offset, 1.0f));

    const std::array<std::uint8_t, 4> expected{
        0x00, 0x00, 0x80, 0x3F
    };

    EXPECT_EQ(buffer, expected);
    EXPECT_EQ(offset, 4u);
}

TEST(HilSerializerTests, WriteF32Negative)
{
    std::array<std::uint8_t, 4> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeF32(buffer, offset, -2.5f));

    const std::array<std::uint8_t, 4> expected{
        0x00, 0x00, 0x20, 0xC0
    };

    EXPECT_EQ(buffer, expected);
    EXPECT_EQ(offset, 4u);
}

TEST(HilSerializerTests, ReadF32Positive)
{
    const std::array<std::uint8_t, 4> buffer{
        0x00, 0x00, 0x80, 0x3F
    };

    std::size_t offset = 0;
    float value = 0.0f;

    ASSERT_TRUE(readF32(buffer, offset, value));

    EXPECT_FLOAT_EQ(value, 1.0f);
    EXPECT_EQ(offset, 4u);
}

TEST(HilSerializerTests, ReadF32Negative)
{
    const std::array<std::uint8_t, 4> buffer{
        0x00, 0x00, 0x20, 0xC0
    };

    std::size_t offset = 0;
    float value = 0.0f;

    ASSERT_TRUE(readF32(buffer, offset, value));

    EXPECT_FLOAT_EQ(value, -2.5f);
    EXPECT_EQ(offset, 4u);
}

TEST(HilSerializerTests, F32RoundTrip)
{
    constexpr std::array<float, 8> testValues{
        0.0f,
        -0.0f,
        1.0f,
        -2.5f,
        9.80665f,
        -0.125f,
        1000000.0f,
        std::numeric_limits<float>::max()
    };

    for (float original : testValues)
    {
        std::array<std::uint8_t, 4> buffer{};
        std::size_t writeOffset = 0;

        ASSERT_TRUE(writeF32(buffer, writeOffset, original));
        EXPECT_EQ(writeOffset, 4u);

        float decoded = 0.0f;
        std::size_t readOffset = 0;

        ASSERT_TRUE(readF32(buffer, readOffset, decoded));

        EXPECT_EQ(
            std::bit_cast<std::uint32_t>(decoded),
            std::bit_cast<std::uint32_t>(original)
        );

        EXPECT_EQ(readOffset, 4u);
    }
}

TEST(HilSerializerTests, F32SubnormalRoundTrip)
{
    const float original =
        std::numeric_limits<float>::denorm_min();

    ASSERT_TRUE(std::isfinite(original));

    std::array<std::uint8_t, 4> buffer{};
    std::size_t writeOffset = 0;

    ASSERT_TRUE(writeF32(buffer, writeOffset, original));

    float decoded = 0.0f;
    std::size_t readOffset = 0;

    ASSERT_TRUE(readF32(buffer, readOffset, decoded));

    EXPECT_EQ(
        std::bit_cast<std::uint32_t>(decoded),
        std::bit_cast<std::uint32_t>(original)
    );
}

TEST(HilSerializerTests, F32WriteRejectsNaN)
{
    std::array<std::uint8_t, 4> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = 0;

    float nan = std::numeric_limits<float>::quiet_NaN();

    EXPECT_FALSE(writeF32(buffer, offset, nan));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);
}

TEST(HilSerializerTests, F32WriteRejectsInfinity)
{
    std::array<std::uint8_t, 4> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = 0;

    const float infinity =
        std::numeric_limits<float>::infinity();

    EXPECT_FALSE(writeF32(buffer, offset, infinity));
    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);

    EXPECT_FALSE(writeF32(buffer, offset, -infinity));
    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);
}

TEST(HilSerializerTests, F32ReadRejectsNaN)
{
    // IEEE-754 quiet NaN: 0x7FC00000
    const std::array<std::uint8_t, 4> buffer{
        0x00, 0x00, 0xC0, 0x7F
    };

    std::size_t offset = 0;
    float value = 42.0f;

    EXPECT_FALSE(readF32(buffer, offset, value));

    EXPECT_EQ(offset, 0u);
    EXPECT_FLOAT_EQ(value, 42.0f);
}

TEST(HilSerializerTests, F32ReadRejectsInfinity)
{
    // IEEE-754 positive infinity: 0x7F800000
    const std::array<std::uint8_t, 4> positiveInfinity{
        0x00, 0x00, 0x80, 0x7F
    };

    // IEEE-754 negative infinity: 0xFF800000
    const std::array<std::uint8_t, 4> negativeInfinity{
        0x00, 0x00, 0x80, 0xFF
    };

    std::size_t offset = 0;
    float value = 42.0f;

    EXPECT_FALSE(readF32(positiveInfinity, offset, value));
    EXPECT_EQ(offset, 0u);
    EXPECT_FLOAT_EQ(value, 42.0f);

    EXPECT_FALSE(readF32(negativeInfinity, offset, value));
    EXPECT_EQ(offset, 0u);
    EXPECT_FLOAT_EQ(value, 42.0f);
}

TEST(HilSerializerTests, F32InsufficientWriteCapacity)
{
    std::array<std::uint8_t, 3> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = 0;

    EXPECT_FALSE(writeF32(buffer, offset, 1.0f));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);
}

TEST(HilSerializerTests, F32InsufficientReadCapacity)
{
    const std::array<std::uint8_t, 3> buffer{
        0x00, 0x00, 0x80
    };

    std::size_t offset = 0;
    float value = 42.0f;

    EXPECT_FALSE(readF32(buffer, offset, value));

    EXPECT_EQ(offset, 0u);
    EXPECT_FLOAT_EQ(value, 42.0f);
}

TEST(HilSerializerTests, F32NonzeroOffset)
{
    std::array<std::uint8_t, 8> buffer{};
    buffer.fill(0xAA);

    std::size_t writeOffset = 2;

    ASSERT_TRUE(writeF32(buffer, writeOffset, 1.0f));

    const std::array<std::uint8_t, 8> expected{
        0xAA, 0xAA,
        0x00, 0x00, 0x80, 0x3F,
        0xAA, 0xAA
    };

    EXPECT_EQ(buffer, expected);
    EXPECT_EQ(writeOffset, 6u);

    std::size_t readOffset = 2;
    float value = 0.0f;

    ASSERT_TRUE(readF32(buffer, readOffset, value));

    EXPECT_FLOAT_EQ(value, 1.0f);
    EXPECT_EQ(readOffset, 6u);
}

TEST(HilSerializerTests, F32InvalidOffset)
{
    std::array<std::uint8_t, 8> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = 100;
    float value = 42.0f;

    EXPECT_FALSE(writeF32(buffer, offset, 1.0f));
    EXPECT_EQ(offset, 100u);
    EXPECT_EQ(buffer, original);

    EXPECT_FALSE(readF32(buffer, offset, value));
    EXPECT_EQ(offset, 100u);
    EXPECT_FLOAT_EQ(value, 42.0f);
}

TEST(HilSerializerTests, F32SequentialValues)
{
    std::array<std::uint8_t, 12> buffer{};

    std::size_t writeOffset = 0;

    ASSERT_TRUE(writeF32(buffer, writeOffset, 1.0f));
    ASSERT_TRUE(writeF32(buffer, writeOffset, -2.5f));
    ASSERT_TRUE(writeF32(buffer, writeOffset, 0.5f));

    EXPECT_EQ(writeOffset, 12u);

    const std::array<std::uint8_t, 12> expected{
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x20, 0xC0,
        0x00, 0x00, 0x00, 0x3F
    };

    EXPECT_EQ(buffer, expected);

    std::size_t readOffset = 0;

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;

    ASSERT_TRUE(readF32(buffer, readOffset, a));
    ASSERT_TRUE(readF32(buffer, readOffset, b));
    ASSERT_TRUE(readF32(buffer, readOffset, c));

    EXPECT_FLOAT_EQ(a, 1.0f);
    EXPECT_FLOAT_EQ(b, -2.5f);
    EXPECT_FLOAT_EQ(c, 0.5f);
    EXPECT_EQ(readOffset, 12u);
}

TEST(HilSerializerTests, F32InvalidValueAtNonzeroOffset)
{
    // Two padding bytes followed by IEEE-754 infinity.
    const std::array<std::uint8_t, 6> buffer{
        0xAA, 0xBB,
        0x00, 0x00, 0x80, 0x7F
    };

    std::size_t offset = 2;
    float value = 42.0f;

    EXPECT_FALSE(readF32(buffer, offset, value));

    // The failed decode must not consume four bytes.
    EXPECT_EQ(offset, 2u);
    EXPECT_FLOAT_EQ(value, 42.0f);
}


// ============================================================
// HIL HEADER SERIALIZATION TESTS
// ============================================================

TEST(HilSerializerTests, WriteHeaderLittleEndian)
{
    HilHeader header{};
    header.version = PROTOCOL_VERSION;
    header.messageType =
        static_cast<std::uint8_t>(MessageType::SENSOR_SAMPLE);
    header.payloadLength = 32;
    header.sequence = 42;

    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeHeader(buffer, offset, header));

    const std::array<std::uint8_t, HEADER_SIZE> expected{
        0x01, 0x02, 0x20, 0x00,
        0x2A, 0x00, 0x00, 0x00
    };

    EXPECT_EQ(buffer, expected);
    EXPECT_EQ(offset, HEADER_SIZE);
}

TEST(HilSerializerTests, ReadHeaderLittleEndian)
{
    const std::array<std::uint8_t, HEADER_SIZE> buffer{
        0x01, 0x02, 0x20, 0x00,
        0x2A, 0x00, 0x00, 0x00
    };

    HilHeader header{};
    std::size_t offset = 0;

    ASSERT_TRUE(readHeader(buffer, offset, header));

    EXPECT_EQ(header.version, PROTOCOL_VERSION);
    EXPECT_EQ(
        header.messageType,
        static_cast<std::uint8_t>(MessageType::SENSOR_SAMPLE)
    );
    EXPECT_EQ(header.payloadLength, 32);
    EXPECT_EQ(header.sequence, 42u);
    EXPECT_EQ(offset, HEADER_SIZE);
}

TEST(HilSerializerTests, HeaderRoundTrip)
{
    HilHeader original{};
    original.version = PROTOCOL_VERSION;
    original.messageType =
        static_cast<std::uint8_t>(MessageType::CONTROL_COMMAND);
    original.payloadLength = 17;
    original.sequence = 0x12345678u;

    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    std::size_t writeOffset = 0;

    ASSERT_TRUE(writeHeader(buffer, writeOffset, original));
    EXPECT_EQ(writeOffset, HEADER_SIZE);

    HilHeader decoded{};
    std::size_t readOffset = 0;

    ASSERT_TRUE(readHeader(buffer, readOffset, decoded));

    EXPECT_EQ(decoded.version, original.version);
    EXPECT_EQ(decoded.messageType, original.messageType);
    EXPECT_EQ(decoded.payloadLength, original.payloadLength);
    EXPECT_EQ(decoded.sequence, original.sequence);
    EXPECT_EQ(readOffset, writeOffset);
}

TEST(HilSerializerTests, HeaderNonzeroOffset)
{
    HilHeader original{};
    original.messageType =
        static_cast<std::uint8_t>(MessageType::STATUS);
    original.payloadLength = 4;
    original.sequence = 12345;

    std::array<std::uint8_t, 12> buffer{};
    buffer.fill(0xAA);

    std::size_t writeOffset = 2;

    ASSERT_TRUE(writeHeader(buffer, writeOffset, original));

    EXPECT_EQ(writeOffset, 10u);
    EXPECT_EQ(buffer[0], 0xAA);
    EXPECT_EQ(buffer[1], 0xAA);
    EXPECT_EQ(buffer[10], 0xAA);
    EXPECT_EQ(buffer[11], 0xAA);

    HilHeader decoded{};
    std::size_t readOffset = 2;

    ASSERT_TRUE(readHeader(buffer, readOffset, decoded));

    EXPECT_EQ(decoded.version, original.version);
    EXPECT_EQ(decoded.messageType, original.messageType);
    EXPECT_EQ(decoded.payloadLength, original.payloadLength);
    EXPECT_EQ(decoded.sequence, original.sequence);
    EXPECT_EQ(readOffset, 10u);
}

TEST(HilSerializerTests, HeaderExactCapacity)
{
    HilHeader header{};
    header.messageType =
        static_cast<std::uint8_t>(MessageType::HELLO);
    header.payloadLength = 0;
    header.sequence = 1;

    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeHeader(buffer, offset, header));
    EXPECT_EQ(offset, buffer.size());

    offset = 0;

    HilHeader decoded{};
    ASSERT_TRUE(readHeader(buffer, offset, decoded));

    EXPECT_EQ(offset, buffer.size());
    EXPECT_EQ(decoded.sequence, 1u);
}

TEST(HilSerializerTests, HeaderInsufficientWriteCapacity)
{
    HilHeader header{};
    header.messageType =
        static_cast<std::uint8_t>(MessageType::SENSOR_SAMPLE);
    header.payloadLength = 32;
    header.sequence = 42;

    std::array<std::uint8_t, HEADER_SIZE - 1> buffer{};
    buffer.fill(0xA5);

    const auto original = buffer;

    std::size_t offset = 0;

    EXPECT_FALSE(writeHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);
}

TEST(HilSerializerTests, HeaderInsufficientReadCapacity)
{
    const std::array<std::uint8_t, HEADER_SIZE - 1> buffer{
        0x01, 0x02, 0x20, 0x00,
        0x2A, 0x00, 0x00
    };

    HilHeader header{};
    header.messageType =
        static_cast<std::uint8_t>(MessageType::FAULT);
    header.payloadLength = 3;
    header.sequence = 999;

    std::size_t offset = 0;

    EXPECT_FALSE(readHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(header.version, PROTOCOL_VERSION);
    EXPECT_EQ(
        header.messageType,
        static_cast<std::uint8_t>(MessageType::FAULT)
    );
    EXPECT_EQ(header.payloadLength, 3);
    EXPECT_EQ(header.sequence, 999u);
}

TEST(HilSerializerTests, HeaderOffsetAtBufferEnd)
{
    HilHeader header{};
    header.messageType =
        static_cast<std::uint8_t>(MessageType::HELLO);

    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = buffer.size();

    EXPECT_FALSE(writeHeader(buffer, offset, header));
    EXPECT_EQ(buffer, original);
    EXPECT_EQ(offset, HEADER_SIZE);

    HilHeader decoded{};
    decoded.sequence = 123;

    EXPECT_FALSE(readHeader(buffer, offset, decoded));

    EXPECT_EQ(offset, HEADER_SIZE);
    EXPECT_EQ(decoded.sequence, 123u);
    EXPECT_EQ(decoded.messageType, 0);
}

TEST(HilSerializerTests, HeaderOffsetBeyondBufferEnd)
{
    HilHeader header{};
    header.messageType =
        static_cast<std::uint8_t>(MessageType::HELLO);

    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = 100;

    EXPECT_FALSE(writeHeader(buffer, offset, header));

    EXPECT_EQ(offset, 100u);
    EXPECT_EQ(buffer, original);

    HilHeader decoded{};
    decoded.sequence = 456;

    EXPECT_FALSE(readHeader(buffer, offset, decoded));

    EXPECT_EQ(offset, 100u);
    EXPECT_EQ(decoded.sequence, 456u);
    EXPECT_EQ(decoded.messageType, 0);
}

TEST(HilSerializerTests, HeaderRejectsUnsupportedVersionOnWrite)
{
    HilHeader header{};
    header.version = 2;
    header.messageType =
        static_cast<std::uint8_t>(MessageType::SENSOR_SAMPLE);
    header.payloadLength = 32;

    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;
    std::size_t offset = 0;

    EXPECT_FALSE(writeHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);
}

TEST(HilSerializerTests, HeaderRejectsUnsupportedVersionOnRead)
{
    const std::array<std::uint8_t, HEADER_SIZE> buffer{
        0x02, 0x02, 0x20, 0x00,
        0x2A, 0x00, 0x00, 0x00
    };

    HilHeader header{};
    header.sequence = 999;

    std::size_t offset = 0;

    EXPECT_FALSE(readHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(header.version, PROTOCOL_VERSION);
    EXPECT_EQ(header.messageType, 0);
    EXPECT_EQ(header.sequence, 999u);
}

TEST(HilSerializerTests, HeaderRejectsInvalidMessageTypesOnWrite)
{
    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    HilHeader header{};
    header.payloadLength = 0;

    std::size_t offset = 0;

    // Type 0 is reserved as invalid.
    header.messageType = 0;

    EXPECT_FALSE(writeHeader(buffer, offset, header));
    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);

    // Type 6 is outside the five defined message types.
    header.messageType = 6;

    EXPECT_FALSE(writeHeader(buffer, offset, header));
    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);
}

TEST(HilSerializerTests, HeaderRejectsInvalidMessageTypesOnRead)
{
    std::array<std::uint8_t, HEADER_SIZE> buffer{
        0x01, 0x00, 0x00, 0x00,
        0x2A, 0x00, 0x00, 0x00
    };

    HilHeader header{};
    header.sequence = 999;

    std::size_t offset = 0;

    // Invalid message type 0.
    EXPECT_FALSE(readHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(header.messageType, 0);
    EXPECT_EQ(header.sequence, 999u);

    // Invalid message type 6.
    buffer[1] = 0x06;

    EXPECT_FALSE(readHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(header.messageType, 0);
    EXPECT_EQ(header.sequence, 999u);
}

TEST(HilSerializerTests, HeaderRejectsOversizedPayloadOnWrite)
{
    HilHeader header{};
    header.messageType =
        static_cast<std::uint8_t>(MessageType::SENSOR_SAMPLE);
    header.payloadLength =
        static_cast<std::uint16_t>(MAX_PAYLOAD_SIZE + 1);

    std::array<std::uint8_t, HEADER_SIZE> buffer{};
    buffer.fill(0xAA);

    const auto original = buffer;

    std::size_t offset = 0;

    EXPECT_FALSE(writeHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(buffer, original);
}

TEST(HilSerializerTests, HeaderRejectsOversizedPayloadOnRead)
{
    // 117-byte payload, exceeding the 116-byte limit.
    const std::array<std::uint8_t, HEADER_SIZE> buffer{
        0x01, 0x02, 0x75, 0x00,
        0x2A, 0x00, 0x00, 0x00
    };

    HilHeader header{};
    header.sequence = 999;

    std::size_t offset = 0;

    EXPECT_FALSE(readHeader(buffer, offset, header));

    EXPECT_EQ(offset, 0u);
    EXPECT_EQ(header.payloadLength, 0);
    EXPECT_EQ(header.sequence, 999u);
}

TEST(HilSerializerTests, HeaderMaximumPayloadAccepted)
{
    HilHeader original{};
    original.messageType =
        static_cast<std::uint8_t>(MessageType::STATUS);
    original.payloadLength =
        static_cast<std::uint16_t>(MAX_PAYLOAD_SIZE);
    original.sequence = 500;

    std::array<std::uint8_t, HEADER_SIZE> buffer{};

    std::size_t writeOffset = 0;

    ASSERT_TRUE(writeHeader(buffer, writeOffset, original));

    EXPECT_EQ(writeOffset, HEADER_SIZE);

    HilHeader decoded{};
    std::size_t readOffset = 0;

    ASSERT_TRUE(readHeader(buffer, readOffset, decoded));

    EXPECT_EQ(decoded.payloadLength, MAX_PAYLOAD_SIZE);
    EXPECT_EQ(decoded.sequence, 500u);
    EXPECT_EQ(readOffset, HEADER_SIZE);
}

TEST(HilSerializerTests, HeaderReadDoesNotRequirePayloadBytes)
{
    // A header may declare a 32-byte payload even when the
    // supplied span contains only the eight header bytes.
    //
    // This is intentional: readHeader() validates only the
    // header. Complete-packet validation happens later.

    const std::array<std::uint8_t, HEADER_SIZE> buffer{
        0x01, 0x02, 0x20, 0x00,
        0x2A, 0x00, 0x00, 0x00
    };

    HilHeader header{};
    std::size_t offset = 0;

    ASSERT_TRUE(readHeader(buffer, offset, header));

    EXPECT_EQ(header.payloadLength, 32);
    EXPECT_EQ(offset, HEADER_SIZE);
}