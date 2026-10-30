#include <gtest/gtest.h>

#include <hil/HilCobs.h>
#include <hil/HilMessage.h>

#include <array>
#include <cstdint>
#include <vector>

using namespace aeroforge::hil;

TEST(HilCobsTests, EmptyRawInput)
{
    const std::array<std::uint8_t, 0> raw{};
    std::array<std::uint8_t, 1> encoded{};

    std::size_t size = 999;

    ASSERT_TRUE(cobsEncode(raw, encoded, size));

    EXPECT_EQ(size, 1u);
    EXPECT_EQ(encoded[0], 0x01);
}

TEST(HilCobsTests, SingleZero)
{
    const std::array<std::uint8_t, 1> raw{ 0x00 };
    std::array<std::uint8_t, 2> encoded{};

    std::size_t size = 0;
    ASSERT_TRUE(cobsEncode(raw, encoded, size));

    EXPECT_EQ(size, 2u);
    EXPECT_EQ(encoded[0], 0x01);
    EXPECT_EQ(encoded[1], 0x01);
}

TEST(HilCobsTests, KnownExample)
{
    const std::array<std::uint8_t, 3> raw{
        0x11, 0x00, 0x22
    };

    std::array<std::uint8_t, 4> encoded{};
    std::size_t size = 0;

    ASSERT_TRUE(cobsEncode(raw, encoded, size));

    const std::array<std::uint8_t, 4> expected{
        0x02, 0x11, 0x02, 0x22
    };

    EXPECT_EQ(size, 4u);
    EXPECT_EQ(encoded, expected);
}

TEST(HilCobsTests, TrailingZero)
{
    const std::array<std::uint8_t, 3> raw{
        0x11, 0x22, 0x00
    };

    std::array<std::uint8_t, 4> encoded{};
    std::size_t size = 0;

    ASSERT_TRUE(cobsEncode(raw, encoded, size));

    const std::array<std::uint8_t, 4> expected{
        0x03, 0x11, 0x22, 0x01
    };

    EXPECT_EQ(encoded, expected);
    EXPECT_EQ(size, 4u);
}

TEST(HilCobsTests, ConsecutiveZeros)
{
    const std::array<std::uint8_t, 2> raw{
        0x00, 0x00
    };

    std::array<std::uint8_t, 3> encoded{};
    std::size_t size = 0;

    ASSERT_TRUE(cobsEncode(raw, encoded, size));

    const std::array<std::uint8_t, 3> expected{
        0x01, 0x01, 0x01
    };

    EXPECT_EQ(encoded, expected);
}

TEST(HilCobsTests, DecodeKnownExample)
{
    const std::array<std::uint8_t, 4> encoded{
        0x02, 0x11, 0x02, 0x22
    };

    std::array<std::uint8_t, 3> decoded{};
    std::size_t size = 0;

    ASSERT_TRUE(cobsDecode(encoded, decoded, size));

    const std::array<std::uint8_t, 3> expected{
        0x11, 0x00, 0x22
    };

    EXPECT_EQ(decoded, expected);
    EXPECT_EQ(size, 3u);
}

TEST(HilCobsTests, MaximumRawSizeRoundTrip)
{
    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> raw{};

    for (std::size_t i = 0; i < raw.size(); ++i)
        raw[i] = static_cast<std::uint8_t>(i);

    std::array<std::uint8_t, MAX_COBS_ENCODED_SIZE> encoded{};
    std::size_t encodedSize = 0;

    ASSERT_TRUE(cobsEncode(raw, encoded, encodedSize));

    for (std::size_t i = 0; i < encodedSize; ++i)
        EXPECT_NE(encoded[i], 0x00);

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> decoded{};
    std::size_t decodedSize = 0;

    ASSERT_TRUE(cobsDecode(
        std::span<const std::uint8_t>(
            encoded.data(), encodedSize),
        decoded,
        decodedSize));

    EXPECT_EQ(decodedSize, raw.size());
    EXPECT_EQ(decoded, raw);
}

TEST(HilCobsTests, RoundTripVariedPayloadLengths)
{
    for (std::size_t length = 0;
        length <= MAX_RAW_PACKET_SIZE;
        ++length)
    {
        std::vector<std::uint8_t> raw(length);

        for (std::size_t i = 0; i < length; ++i)
        {
            raw[i] = static_cast<std::uint8_t>(
                (i * 37) & 0xFF);
        }

        std::array<std::uint8_t, MAX_COBS_ENCODED_SIZE>
            encoded{};

        std::size_t encodedSize = 0;

        ASSERT_TRUE(cobsEncode(
            raw, encoded, encodedSize));

        std::array<std::uint8_t, MAX_RAW_PACKET_SIZE>
            decoded{};

        std::size_t decodedSize = 0;

        ASSERT_TRUE(cobsDecode(
            std::span<const std::uint8_t>(
                encoded.data(), encodedSize),
            decoded, decodedSize));

        ASSERT_EQ(decodedSize, length);

        for (std::size_t i = 0; i < length; ++i)
            EXPECT_EQ(decoded[i], raw[i]);
    }
}

TEST(HilCobsTests, RejectsZeroCode)
{
    const std::array<std::uint8_t, 2> encoded{
        0x00, 0x11
    };

    std::array<std::uint8_t, 10> output{};
    output.fill(0xA5);

    const auto original = output;
    std::size_t size = 777;

    EXPECT_FALSE(cobsDecode(encoded, output, size));

    EXPECT_EQ(output, original);
    EXPECT_EQ(size, 777u);
}

TEST(HilCobsTests, RejectsTruncatedBlock)
{
    const std::array<std::uint8_t, 2> encoded{
        0x04, 0x11
    };

    std::array<std::uint8_t, 10> output{};
    output.fill(0xA5);

    const auto original = output;
    std::size_t size = 777;

    EXPECT_FALSE(cobsDecode(encoded, output, size));

    EXPECT_EQ(output, original);
    EXPECT_EQ(size, 777u);
}

TEST(HilCobsTests, RejectsZeroInsideLiteralBlock)
{
    const std::array<std::uint8_t, 3> encoded{
        0x03, 0x11, 0x00
    };

    std::array<std::uint8_t, 10> output{};
    std::size_t size = 0;

    EXPECT_FALSE(cobsDecode(encoded, output, size));
}

TEST(HilCobsTests, EncodeFailurePreservesOutput)
{
    const std::array<std::uint8_t, 3> raw{
        0x11, 0x00, 0x22
    };

    std::array<std::uint8_t, 3> output{};
    output.fill(0xA5);

    const auto original = output;
    std::size_t size = 555;

    EXPECT_FALSE(cobsEncode(raw, output, size));

    EXPECT_EQ(output, original);
    EXPECT_EQ(size, 555u);
}

TEST(HilCobsTests, DecodeFailurePreservesOutput)
{
    const std::array<std::uint8_t, 4> encoded{
        0x02, 0x11, 0x02, 0x22
    };

    std::array<std::uint8_t, 2> output{};
    output.fill(0xA5);

    const auto original = output;
    std::size_t size = 555;

    EXPECT_FALSE(cobsDecode(encoded, output, size));

    EXPECT_EQ(output, original);
    EXPECT_EQ(size, 555u);
}

TEST(HilCobsTests, RejectsOversizedRawInput)
{
    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE + 1>
        raw{};

    std::array<std::uint8_t, MAX_COBS_ENCODED_SIZE + 2>
        output{};

    std::size_t size = 0;

    EXPECT_FALSE(cobsEncode(raw, output, size));
}

TEST(HilCobsTests, RejectsOversizedEncodedInput)
{
    std::array<std::uint8_t, MAX_COBS_ENCODED_SIZE + 1>
        encoded{};

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE>
        decoded{};

    std::size_t size = 0;

    EXPECT_FALSE(cobsDecode(encoded, decoded, size));
}