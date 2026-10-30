#include <gtest/gtest.h>
#include <hil/HilTelemetryStore.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace aeroforge::hil;

TEST(HilTelemetryStoreTests, EmptySnapshot)
{
    HilTelemetryStore store;
    const auto s = store.snapshot();
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.health.mode, HilLinkMode::OFFLINE);
}

TEST(HilTelemetryStoreTests, PreservesOrder)
{
    HilTelemetryStore store;
    HilHealthSnapshot health{};
    for (int i = 0; i < 3; ++i)
        store.publish({ static_cast<double>(i), 0.1 * i, 0.02, -1 }, health);
    const auto s = store.snapshot();
    ASSERT_EQ(s.count, 3u);
    EXPECT_DOUBLE_EQ(s.points[0].simulationTimeS, 0.0);
    EXPECT_DOUBLE_EQ(s.points[2].simulationTimeS, 2.0);
}

TEST(HilTelemetryStoreTests, BoundedHistoryKeepsNewest)
{
    HilTelemetryStore store;
    HilHealthSnapshot health{};
    constexpr auto capacity = HilTelemetrySnapshot::CAPACITY;
    for (std::size_t i = 0; i < capacity + 23; ++i)
        store.publish({ static_cast<double>(i), 0.1, 0.01, -1.0 }, health);
    const auto s = store.snapshot();
    ASSERT_EQ(s.count, capacity);
    EXPECT_DOUBLE_EQ(s.points.front().simulationTimeS, 23.0);
    EXPECT_DOUBLE_EQ(s.points[capacity - 1].simulationTimeS,
        static_cast<double>(capacity + 22));
}

TEST(HilTelemetryStoreTests, CopiesHealthAndIncrementsRevision)
{
    HilTelemetryStore store;
    HilHealthSnapshot h{};
    h.mode = HilLinkMode::SOFTWARE_LOOPBACK;
    h.acceptedCommands = 87;
    store.publishHealth(h);
    const auto a = store.snapshot();
    EXPECT_EQ(a.count, 0u);
    EXPECT_EQ(a.health.acceptedCommands, 87u);
    EXPECT_EQ(a.health.mode, HilLinkMode::SOFTWARE_LOOPBACK);
    store.publish({ 1.0, 0.1, 0.02, 0.05 }, h);
    EXPECT_GT(store.snapshot().revision, a.revision);
}

TEST(HilTelemetryStoreTests, RejectsInvalidSamplesWithoutMutation)
{
    HilTelemetryStore store;
    EXPECT_THROW(store.publish({ -1.0, 0.0, 0.0, -1.0 }, {}), std::invalid_argument);
    EXPECT_THROW(store.publish({ 1.0, -2.0, 0.0, -1.0 }, {}), std::invalid_argument);
    EXPECT_THROW(store.publish({ 1.0, 0.0,
        std::numeric_limits<double>::quiet_NaN(), -1.0 }, {}), std::invalid_argument);
    EXPECT_THROW(store.publish({ 1.0, 0.0, 0.0, -2.0 }, {}), std::invalid_argument);
    EXPECT_EQ(store.snapshot().count, 0u);
}

TEST(HilTelemetryStoreTests, ResetClearsSamplesAndHealth)
{
    HilTelemetryStore store;
    HilHealthSnapshot health{};
    health.acceptedCommands = 100;
    store.publish({ 1.0, 0.01, 0.02, -1.0 }, health);
    store.reset();
    const auto s = store.snapshot();
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.health.acceptedCommands, 0u);
    EXPECT_EQ(s.health.mode, HilLinkMode::OFFLINE);
}

TEST(HilTelemetryStoreTests, ConcurrentPublisherAndReaderMaintainOrderedSnapshots)
{
    HilTelemetryStore store;
    std::atomic<bool> invalid{ false };
    std::atomic<bool> finished{ false };
    std::thread producer([&]
        {
            for (std::size_t i = 0; i < 3000; ++i)
                store.publish({ static_cast<double>(i), 0.1, 0.02, -1.0 }, {});
            finished = true;
        });
    while (!finished.load())
    {
        const auto s = store.snapshot();
        for (std::size_t i = 1; i < s.count; ++i)
            if (!(s.points[i].simulationTimeS > s.points[i - 1].simulationTimeS))
                invalid = true;
    }
    producer.join();
    EXPECT_FALSE(invalid.load());
    EXPECT_EQ(store.snapshot().count, HilTelemetrySnapshot::CAPACITY);
}
