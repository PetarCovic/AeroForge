#include <gtest/gtest.h>

#include <hil/HilStreamParser.h>
#include <hil/HilCobs.h>

#include <array>
#include <cstdint>
#include <vector>
#include <span>

using namespace aeroforge::hil;

namespace
{
    std::vector<std::uint8_t> makeFrame(
        std::span<const std::uint8_t> raw)
    {
        std::array<std::uint8_t, MAX_COBS_ENCODED_SIZE>
            encoded{};

        std::size_t encodedSize = 0;

        if (!cobsEncode(raw, encoded, encodedSize))
            return {};

        std::vector<std::uint8_t> frame(
            encoded.begin(),
            encoded.begin() + encodedSize);

        frame.push_back(0x00);
        return frame;
    }
}

TEST(HilStreamParserTests, CompleteFrame)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 3> raw{
        0x11, 0x00, 0x22
    };

    const auto frame = makeFrame(raw);

    parser.consume(frame);

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> output{};
    std::size_t size = 0;

    ASSERT_TRUE(parser.tryPopPacket(output, size));

    EXPECT_EQ(size, 3u);
    EXPECT_EQ(output[0], 0x11);
    EXPECT_EQ(output[1], 0x00);
    EXPECT_EQ(output[2], 0x22);
}

TEST(HilStreamParserTests, OneByteAtATime)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 4> raw{
        0x11, 0x00, 0x22, 0x33
    };

    const auto frame = makeFrame(raw);

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> output{};
    std::size_t size = 0;

    for (std::size_t i = 0; i < frame.size(); ++i)
    {
        parser.consume(
            std::span<const std::uint8_t>(
                &frame[i], 1));

        if (i + 1 < frame.size())
            EXPECT_FALSE(parser.tryPopPacket(output, size));
    }

    ASSERT_TRUE(parser.tryPopPacket(output, size));

    EXPECT_EQ(size, raw.size());

    for (std::size_t i = 0; i < raw.size(); ++i)
        EXPECT_EQ(output[i], raw[i]);
}

TEST(HilStreamParserTests, EveryPossibleSplitPoint)
{
    const std::array<std::uint8_t, 6> raw{
        0, 1, 2, 0, 3, 4
    };

    const auto frame = makeFrame(raw);

    for (std::size_t split = 1;
        split < frame.size();
        ++split)
    {
        HilStreamParser parser;

        parser.consume(std::span<const std::uint8_t>(
            frame.data(), split));

        std::array<std::uint8_t, MAX_RAW_PACKET_SIZE>
            output{};

        std::size_t size = 0;

        EXPECT_FALSE(parser.tryPopPacket(output, size));

        parser.consume(std::span<const std::uint8_t>(
            frame.data() + split, frame.size() - split));

        ASSERT_TRUE(parser.tryPopPacket(output, size));
        ASSERT_EQ(size, raw.size());

        for (std::size_t i = 0; i < raw.size(); ++i)
            EXPECT_EQ(output[i], raw[i]);
    }
}

TEST(HilStreamParserTests, ConcatenatedFrames)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 2> a{ 0x11, 0x22 };
    const std::array<std::uint8_t, 3> b{ 0x33, 0, 0x44 };

    const auto frameA = makeFrame(a);
    const auto frameB = makeFrame(b);

    std::vector<std::uint8_t> stream = frameA;

    stream.insert(
        stream.end(), frameB.begin(), frameB.end());

    parser.consume(stream);

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> output{};
    std::size_t size = 0;

    ASSERT_TRUE(parser.tryPopPacket(output, size));
    EXPECT_EQ(size, 2u);
    EXPECT_EQ(output[0], 0x11);

    ASSERT_TRUE(parser.tryPopPacket(output, size));
    EXPECT_EQ(size, 3u);
    EXPECT_EQ(output[0], 0x33);
    EXPECT_EQ(output[1], 0x00);
    EXPECT_EQ(output[2], 0x44);

    EXPECT_FALSE(parser.tryPopPacket(output, size));
}

TEST(HilStreamParserTests, InvalidFrameThenValidFrame)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 3> malformed{
        0x04, 0x11, 0x00
    };

    parser.consume(malformed);

    const std::array<std::uint8_t, 2> raw{
        0xAA, 0xBB
    };

    parser.consume(makeFrame(raw));

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> output{};
    std::size_t size = 0;

    ASSERT_TRUE(parser.tryPopPacket(output, size));

    EXPECT_EQ(size, 2u);
    EXPECT_EQ(output[0], 0xAA);
    EXPECT_EQ(output[1], 0xBB);
    EXPECT_EQ(parser.stats().malformedFrames, 1u);
}

TEST(HilStreamParserTests, OversizedFrameRecovers)
{
    HilStreamParser parser;

    std::vector<std::uint8_t> oversized(
        MAX_COBS_ENCODED_SIZE + 1, 0x11);

    oversized.push_back(0x00);

    parser.consume(oversized);

    EXPECT_EQ(parser.stats().oversizedFrames, 1u);

    const std::array<std::uint8_t, 2> raw{
        0xAA, 0xBB
    };

    parser.consume(makeFrame(raw));

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> output{};
    std::size_t size = 0;

    ASSERT_TRUE(parser.tryPopPacket(output, size));

    EXPECT_EQ(size, raw.size());
    EXPECT_EQ(output[0], 0xAA);
}

TEST(HilStreamParserTests, QueueOverflowIsBounded)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 1> raw{ 0xAA };
    const auto frame = makeFrame(raw);

    for (std::size_t i = 0;
        i < HilStreamParser::QUEUE_CAPACITY + 1;
        ++i)
    {
        parser.consume(frame);
    }

    EXPECT_EQ(
        parser.stats().completedFrames,
        HilStreamParser::QUEUE_CAPACITY);

    EXPECT_EQ(parser.stats().droppedFrames, 1u);
}

TEST(HilStreamParserTests, SmallOutputDoesNotConsumePacket)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 4> raw{
        1, 2, 3, 4
    };

    parser.consume(makeFrame(raw));

    std::array<std::uint8_t, 2> small{};
    small.fill(0xA5);

    const auto original = small;
    std::size_t size = 999;

    EXPECT_FALSE(parser.tryPopPacket(small, size));
    EXPECT_EQ(small, original);
    EXPECT_EQ(size, 999u);

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE>
        sufficient{};

    ASSERT_TRUE(parser.tryPopPacket(sufficient, size));

    EXPECT_EQ(size, 4u);
    EXPECT_EQ(sufficient[0], 1);
}

TEST(HilStreamParserTests, ResetClearsBufferedState)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 3> partial{
        0x03, 0x11, 0x22
    };

    parser.consume(partial);
    parser.reset();

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> output{};
    std::size_t size = 0;

    EXPECT_FALSE(parser.tryPopPacket(output, size));
    EXPECT_EQ(parser.stats().completedFrames, 0u);
}

TEST(HilStreamParserTests, EmptyDelimitersAreCounted)
{
    HilStreamParser parser;

    const std::array<std::uint8_t, 3> bytes{
        0x00, 0x00, 0x00
    };

    parser.consume(bytes);

    EXPECT_EQ(parser.stats().emptyFrames, 3u);
}