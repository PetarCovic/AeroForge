
#include <gtest/gtest.h>

#include <hil/LoopbackTransport.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <thread>
#include <vector>

using namespace aeroforge::hil;

namespace
{
    std::vector<std::uint8_t> makePayload(
        std::size_t count,
        std::uint8_t seed = 0)
    {
        std::vector<std::uint8_t> result(count);

        for (std::size_t i = 0; i < count; ++i)
        {
            result[i] = static_cast<std::uint8_t>(
                (i * 37 + seed) % 256
                );
        }

        return result;
    }

    using Deadline =
        std::chrono::steady_clock::time_point;

    std::size_t writeUntil(
        LoopbackTransport& transport,
        std::span<const std::uint8_t> data,
        Deadline deadline)
    {
        std::size_t totalWritten = 0;

        while (totalWritten < data.size() &&
            std::chrono::steady_clock::now() < deadline)
        {
            const std::size_t written = transport.tryWrite(
                data.subspan(totalWritten)
            );

            totalWritten += written;

            if (written == 0)
            {
                std::this_thread::yield();
            }
        }

        return totalWritten;
    }

    std::vector<std::uint8_t> readUntil(
        LoopbackTransport& transport,
        std::size_t expectedCount,
        Deadline deadline)
    {
        std::vector<std::uint8_t> received;
        received.reserve(expectedCount);

        std::array<std::uint8_t, 127> buffer{};

        while (received.size() < expectedCount &&
            std::chrono::steady_clock::now() < deadline)
        {
            const std::size_t remaining =
                expectedCount - received.size();

            auto view = std::span<std::uint8_t>(buffer).first(
                std::min(buffer.size(), remaining)
            );

            const std::size_t count = transport.tryRead(view);

            for (std::size_t i = 0; i < count; ++i)
            {
                received.push_back(buffer[i]);
            }

            if (count == 0)
            {
                std::this_thread::yield();
            }
        }

        return received;
    }
}

// ============================================================
// 1. Factory and endpoint identity
// ============================================================

TEST(LoopbackTransportTests, CreatePair)
{
    auto [a, b] = LoopbackTransport::createPair();

    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);

    EXPECT_TRUE(a->isOpen());
    EXPECT_TRUE(b->isOpen());

    EXPECT_EQ(a->getEndpoint(), Endpoint::A);
    EXPECT_EQ(b->getEndpoint(), Endpoint::B);

    EXPECT_NE(a.get(), b.get());
}

// ============================================================
// 2. Basic communication
// ============================================================

TEST(LoopbackTransportTests, WriteAToB)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 3> sent{ 10, 20, 30 };
    std::array<std::uint8_t, 3> received{};

    EXPECT_EQ(a->tryWrite(sent), sent.size());
    EXPECT_EQ(b->tryRead(received), received.size());

    EXPECT_EQ(received, sent);
}

TEST(LoopbackTransportTests, WriteBToA)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 4> sent{ 40, 50, 60, 70 };
    std::array<std::uint8_t, 4> received{};

    EXPECT_EQ(b->tryWrite(sent), sent.size());
    EXPECT_EQ(a->tryRead(received), received.size());

    EXPECT_EQ(received, sent);
}

TEST(LoopbackTransportTests, IndependentDirections)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 3> aData{ 1, 2, 3 };
    std::array<std::uint8_t, 3> bData{ 4, 5, 6 };

    std::array<std::uint8_t, 3> receivedByA{};
    std::array<std::uint8_t, 3> receivedByB{};

    EXPECT_EQ(a->tryWrite(aData), aData.size());
    EXPECT_EQ(b->tryWrite(bData), bData.size());

    EXPECT_EQ(a->tryRead(receivedByA), bData.size());
    EXPECT_EQ(b->tryRead(receivedByB), aData.size());

    EXPECT_EQ(receivedByA, bData);
    EXPECT_EQ(receivedByB, aData);
}

// ============================================================
// 3. Empty and partial transfers
// ============================================================

TEST(LoopbackTransportTests, EmptyRead)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 8> buffer{};

    EXPECT_EQ(a->tryRead(buffer), 0u);
    EXPECT_EQ(b->tryRead(buffer), 0u);
}

TEST(LoopbackTransportTests, EmptyWrite)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::span<const std::uint8_t> empty;

    EXPECT_EQ(a->tryWrite(empty), 0u);
    EXPECT_EQ(b->tryWrite(empty), 0u);

    std::array<std::uint8_t, 8> buffer{};

    EXPECT_EQ(a->tryRead(buffer), 0u);
    EXPECT_EQ(b->tryRead(buffer), 0u);
}

TEST(LoopbackTransportTests, ZeroLengthReadPreservesData)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 3> sent{ 10, 20, 30 };

    ASSERT_EQ(a->tryWrite(sent), sent.size());

    std::span<std::uint8_t> empty;

    EXPECT_EQ(b->tryRead(empty), 0u);

    std::array<std::uint8_t, 3> received{};

    EXPECT_EQ(b->tryRead(received), sent.size());
    EXPECT_EQ(received, sent);
}

TEST(LoopbackTransportTests, PartialRead)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 5> sent{
        10, 20, 30, 40, 50
    };

    ASSERT_EQ(a->tryWrite(sent), sent.size());

    std::array<std::uint8_t, 3> first{};
    std::array<std::uint8_t, 3> second{};

    ASSERT_EQ(b->tryRead(first), 3u);

    EXPECT_EQ(
        first,
        (std::array<std::uint8_t, 3>{10, 20, 30})
    );

    ASSERT_EQ(b->tryRead(second), 2u);

    EXPECT_EQ(second[0], 40);
    EXPECT_EQ(second[1], 50);

    EXPECT_EQ(b->tryRead(second), 0u);
}

TEST(LoopbackTransportTests, FifoOrdering)
{
    auto [a, b] = LoopbackTransport::createPair();

    const auto sent = makePayload(1000);

    std::vector<std::uint8_t> received(sent.size());

    ASSERT_EQ(a->tryWrite(sent), sent.size());
    ASSERT_EQ(b->tryRead(received), sent.size());

    EXPECT_EQ(received, sent);
}

// ============================================================
// 4. Queue capacity and partial writes
// ============================================================

TEST(LoopbackTransportTests, QueueCapacity)
{
    auto [a, b] = LoopbackTransport::createPair();

    constexpr std::size_t capacity =
        LoopbackTransport::MAX_CAPACITY;

    const auto sent = makePayload(capacity);

    std::vector<std::uint8_t> received(capacity);

    EXPECT_EQ(a->tryWrite(sent), capacity);
    EXPECT_EQ(b->tryRead(received), capacity);

    EXPECT_EQ(received, sent);
}

TEST(LoopbackTransportTests, PartialWrite)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::vector<std::uint8_t> initial(4000, 0x11);
    std::vector<std::uint8_t> additional(200, 0x22);

    ASSERT_EQ(a->tryWrite(initial), 4000u);

    EXPECT_EQ(a->tryWrite(additional), 96u);

    std::vector<std::uint8_t> received(
        LoopbackTransport::MAX_CAPACITY
    );

    ASSERT_EQ(
        b->tryRead(received),
        LoopbackTransport::MAX_CAPACITY
    );

    EXPECT_TRUE(std::all_of(
        received.begin(),
        received.begin() + 4000,
        [](std::uint8_t value) {
            return value == 0x11;
        }
    ));

    EXPECT_TRUE(std::all_of(
        received.begin() + 4000,
        received.end(),
        [](std::uint8_t value) {
            return value == 0x22;
        }
    ));
}

TEST(LoopbackTransportTests, FullQueueRejectsAdditionalBytes)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::vector<std::uint8_t> full(
        LoopbackTransport::MAX_CAPACITY,
        0xAA
    );

    std::array<std::uint8_t, 3> extra{ 1, 2, 3 };

    ASSERT_EQ(
        a->tryWrite(full),
        LoopbackTransport::MAX_CAPACITY
    );

    EXPECT_EQ(a->tryWrite(extra), 0u);
}

TEST(LoopbackTransportTests, ReadingRestoresCapacity)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::vector<std::uint8_t> full(
        LoopbackTransport::MAX_CAPACITY,
        0x11
    );

    ASSERT_EQ(
        a->tryWrite(full),
        LoopbackTransport::MAX_CAPACITY
    );

    std::array<std::uint8_t, 100> buffer{};

    ASSERT_EQ(b->tryRead(buffer), 100u);

    std::vector<std::uint8_t> additional(150, 0x22);

    EXPECT_EQ(a->tryWrite(additional), 100u);

    std::vector<std::uint8_t> remaining(
        LoopbackTransport::MAX_CAPACITY
    );

    ASSERT_EQ(
        b->tryRead(remaining),
        LoopbackTransport::MAX_CAPACITY
    );

    EXPECT_TRUE(std::all_of(
        remaining.begin(),
        remaining.begin() + 3996,
        [](std::uint8_t value) {
            return value == 0x11;
        }
    ));

    EXPECT_TRUE(std::all_of(
        remaining.begin() + 3996,
        remaining.end(),
        [](std::uint8_t value) {
            return value == 0x22;
        }
    ));
}

// ============================================================
// 5. Connection shutdown
// ============================================================

TEST(LoopbackTransportTests, LocalClose)
{
    auto [a, b] = LoopbackTransport::createPair();

    a->close();

    EXPECT_FALSE(a->isOpen());
    EXPECT_TRUE(b->isOpen());
}

TEST(LoopbackTransportTests, CloseEndpointB)
{
    auto [a, b] = LoopbackTransport::createPair();

    b->close();

    EXPECT_TRUE(a->isOpen());
    EXPECT_FALSE(b->isOpen());
}

TEST(LoopbackTransportTests, WriteAfterClose)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 3> data{ 1, 2, 3 };

    a->close();

    EXPECT_EQ(a->tryWrite(data), 0u);
    EXPECT_EQ(b->tryWrite(data), 0u);
}

TEST(LoopbackTransportTests, ReadAfterLocalClose)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 3> data{ 1, 2, 3 };

    ASSERT_EQ(b->tryWrite(data), 3u);

    a->close();

    std::array<std::uint8_t, 3> buffer{};

    EXPECT_EQ(a->tryRead(buffer), 0u);
}

TEST(LoopbackTransportTests, DrainAfterPeerClose)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 4> sent{ 10, 20, 30, 40 };

    ASSERT_EQ(a->tryWrite(sent), sent.size());

    a->close();

    EXPECT_FALSE(a->isOpen());
    EXPECT_TRUE(b->isOpen());

    std::array<std::uint8_t, 4> received{};

    EXPECT_EQ(b->tryRead(received), sent.size());
    EXPECT_EQ(received, sent);

    EXPECT_EQ(b->tryRead(received), 0u);
}

TEST(LoopbackTransportTests, RepeatedClose)
{
    auto [a, b] = LoopbackTransport::createPair();

    a->close();
    a->close();
    a->close();

    EXPECT_FALSE(a->isOpen());
    EXPECT_TRUE(b->isOpen());

    b->close();
    b->close();

    EXPECT_FALSE(b->isOpen());
}

// ============================================================
// 6. Destruction and ownership
// ============================================================

TEST(LoopbackTransportTests, DestructorClosesEndpoint)
{
    auto [a, b] = LoopbackTransport::createPair();

    ASSERT_TRUE(a->isOpen());
    ASSERT_TRUE(b->isOpen());

    a.reset();

    EXPECT_TRUE(b->isOpen());

    std::array<std::uint8_t, 3> data{ 1, 2, 3 };

    EXPECT_EQ(b->tryWrite(data), 0u);
}

TEST(LoopbackTransportTests, DestructorPreservesQueuedBytes)
{
    auto [a, b] = LoopbackTransport::createPair();

    std::array<std::uint8_t, 4> sent{ 5, 10, 15, 20 };

    ASSERT_EQ(a->tryWrite(sent), sent.size());

    a.reset();

    std::array<std::uint8_t, 4> received{};

    EXPECT_EQ(b->tryRead(received), sent.size());
    EXPECT_EQ(received, sent);
}

TEST(LoopbackTransportTests, ExplicitCloseBeforeDestruction)
{
    auto [a, b] = LoopbackTransport::createPair();

    a->close();
    a.reset();

    EXPECT_TRUE(b->isOpen());

    std::array<std::uint8_t, 1> data{ 42 };

    EXPECT_EQ(b->tryWrite(data), 0u);
}

// ============================================================
// 7. Concurrent communication
// ============================================================

TEST(LoopbackTransportTests, ConcurrentProducerConsumer)
{
    auto [a, b] = LoopbackTransport::createPair();

    const auto sent = makePayload(20000);

    std::size_t written = 0;
    std::vector<std::uint8_t> received;

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(10);

    {
        std::jthread writer([&]() {
            written = writeUntil(*a, sent, deadline);
            });

        std::jthread reader([&]() {
            received = readUntil(*b, sent.size(), deadline);
            });
    }

    ASSERT_EQ(written, sent.size());
    ASSERT_EQ(received.size(), sent.size());

    EXPECT_EQ(received, sent);
}

TEST(LoopbackTransportTests, ConcurrentBidirectionalTraffic)
{
    auto [a, b] = LoopbackTransport::createPair();

    const auto sentAB = makePayload(16000, 17);
    const auto sentBA = makePayload(16000, 91);

    std::size_t writtenAB = 0;
    std::size_t writtenBA = 0;

    std::vector<std::uint8_t> receivedAB;
    std::vector<std::uint8_t> receivedBA;

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(10);

    {
        std::jthread writerA([&]() {
            writtenAB = writeUntil(*a, sentAB, deadline);
            });

        std::jthread readerB([&]() {
            receivedAB = readUntil(
                *b, sentAB.size(), deadline
            );
            });

        std::jthread writerB([&]() {
            writtenBA = writeUntil(*b, sentBA, deadline);
            });

        std::jthread readerA([&]() {
            receivedBA = readUntil(
                *a, sentBA.size(), deadline
            );
            });
    }

    ASSERT_EQ(writtenAB, sentAB.size());
    ASSERT_EQ(writtenBA, sentBA.size());

    ASSERT_EQ(receivedAB.size(), sentAB.size());
    ASSERT_EQ(receivedBA.size(), sentBA.size());

    EXPECT_EQ(receivedAB, sentAB);
    EXPECT_EQ(receivedBA, sentBA);
}

TEST(LoopbackTransportTests, ConcurrentCloseDuringWrite)
{
    auto [a, b] = LoopbackTransport::createPair();

    const auto sent = makePayload(2000);

    std::size_t written = 0;

    {
        std::jthread writer([&]() {
            written = a->tryWrite(sent);
            });

        a->close();
    }

    EXPECT_FALSE(a->isOpen());
    EXPECT_TRUE(b->isOpen());

    EXPECT_LE(written, sent.size());

    std::vector<std::uint8_t> received(sent.size());

    const std::size_t count = b->tryRead(received);

    EXPECT_EQ(count, written);

    for (std::size_t i = 0; i < count; ++i)
    {
        EXPECT_EQ(received[i], sent[i]);
    }

    EXPECT_EQ(a->tryWrite(sent), 0u);
    EXPECT_EQ(b->tryWrite(sent), 0u);
}
