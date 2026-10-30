#include <gtest/gtest.h>

#include <hil/HilSequenceTracker.h>

#include <cstdint>
#include <limits>

using namespace aeroforge::hil;

TEST(HilSequenceTrackerTests, AcceptsFirstValidCommand)
{
    HilSequenceTracker tracker(100000);

    ASSERT_TRUE(tracker.recordSentSensor(10, 1000000));

    EXPECT_TRUE(tracker.acceptControl(
        20, 10, 1010000));

    ASSERT_TRUE(tracker.lastAcceptedSensorTimeUs().has_value());

    EXPECT_EQ(
        *tracker.lastAcceptedSensorTimeUs(),
        1000000u);
}

TEST(HilSequenceTrackerTests, RejectsDuplicateSensorSequence)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(5, 1000));

    EXPECT_FALSE(tracker.recordSentSensor(5, 2000));
    EXPECT_FALSE(tracker.recordSentSensor(4, 3000));

    EXPECT_TRUE(tracker.recordSentSensor(6, 4000));
}

TEST(HilSequenceTrackerTests, RejectsBackwardSensorTime)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(1, 1000));

    EXPECT_FALSE(tracker.recordSentSensor(2, 999));

    // Failure should not consume sequence 2.
    EXPECT_TRUE(tracker.recordSentSensor(2, 1001));
}

TEST(HilSequenceTrackerTests, RejectsDuplicateControl)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(10, 1000));
    ASSERT_TRUE(tracker.acceptControl(20, 10, 1100));

    EXPECT_FALSE(tracker.acceptControl(20, 10, 1200));
}

TEST(HilSequenceTrackerTests, RejectsOutOfOrderControl)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(10, 1000));
    ASSERT_TRUE(tracker.recordSentSensor(11, 2000));

    ASSERT_TRUE(tracker.acceptControl(20, 10, 2100));

    EXPECT_FALSE(tracker.acceptControl(19, 11, 2200));

    // The rejected command must not consume sequence 21.
    EXPECT_TRUE(tracker.acceptControl(21, 11, 2300));
}

TEST(HilSequenceTrackerTests, RejectsReusedReference)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(10, 1000));
    ASSERT_TRUE(tracker.acceptControl(20, 10, 1100));

    EXPECT_FALSE(tracker.acceptControl(21, 10, 1200));
}

TEST(HilSequenceTrackerTests, RejectsUnknownReference)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(10, 1000));

    EXPECT_FALSE(tracker.acceptControl(20, 999, 1100));

    // An invalid command must not consume command sequence 20.
    EXPECT_TRUE(tracker.acceptControl(20, 10, 1200));
}

TEST(HilSequenceTrackerTests, RejectsStaleCommand)
{
    HilSequenceTracker tracker(50000);

    ASSERT_TRUE(tracker.recordSentSensor(10, 100000));

    EXPECT_FALSE(tracker.acceptControl(
        20, 10, 150001));

    EXPECT_TRUE(tracker.acceptControl(
        20, 10, 150000));
}

TEST(HilSequenceTrackerTests, RejectsFutureTimestampReference)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(10, 100000));

    EXPECT_FALSE(tracker.acceptControl(
        20, 10, 99999));

    EXPECT_TRUE(tracker.acceptControl(
        20, 10, 100000));
}

TEST(HilSequenceTrackerTests, AcceptsSequenceRollover)
{
    HilSequenceTracker tracker;

    const auto max =
        std::numeric_limits<std::uint32_t>::max();

    ASSERT_TRUE(tracker.recordSentSensor(max, 1000));
    ASSERT_TRUE(tracker.recordSentSensor(0, 2000));

    ASSERT_TRUE(tracker.acceptControl(max, max, 2000));

    EXPECT_TRUE(tracker.acceptControl(0, 0, 2100));
}

TEST(HilSequenceTrackerTests, RejectsAmbiguousSequenceJump)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(0, 1000));

    EXPECT_FALSE(tracker.recordSentSensor(
        0x80000000u, 2000));
}

TEST(HilSequenceTrackerTests, EvictsOldSensorReferences)
{
    HilSequenceTracker tracker(1000000);

    for (std::uint32_t sequence = 0;
        sequence < HilSequenceTracker::HISTORY_CAPACITY + 1;
        ++sequence)
    {
        ASSERT_TRUE(tracker.recordSentSensor(
            sequence,
            static_cast<std::uint64_t>(sequence) * 1000));
    }

    EXPECT_FALSE(tracker.acceptControl(
        0, 0, 200000));

    EXPECT_TRUE(tracker.acceptControl(
        0,
        static_cast<std::uint32_t>(
            HilSequenceTracker::HISTORY_CAPACITY),
        200000));
}

TEST(HilSequenceTrackerTests, ResetClearsSessionState)
{
    HilSequenceTracker tracker;

    ASSERT_TRUE(tracker.recordSentSensor(10, 1000));
    ASSERT_TRUE(tracker.acceptControl(20, 10, 1100));

    tracker.reset();

    EXPECT_FALSE(
        tracker.lastAcceptedSensorTimeUs().has_value());

    EXPECT_FALSE(tracker.acceptControl(0, 10, 1200));

    ASSERT_TRUE(tracker.recordSentSensor(0, 0));
    EXPECT_TRUE(tracker.acceptControl(0, 0, 100));
}