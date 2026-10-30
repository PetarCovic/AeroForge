
#include <gtest/gtest.h>

#include <hil/HilCommandWatchdog.h>

#include <chrono>
#include <stdexcept>

using namespace aeroforge::hil;
using namespace std::chrono_literals;

namespace
{
    using Clock = HilCommandWatchdog::Clock;
    const Clock::time_point start{};
}

TEST(HilCommandWatchdogTests, StartsDisarmed)
{
    HilCommandWatchdog watchdog(100ms);

    EXPECT_EQ(
        watchdog.state(start),
        HilWatchdogState::WAITING);

    EXPECT_FALSE(watchdog.permitsCommand(start));
}

TEST(HilCommandWatchdogTests, ValidCommandArmsWatchdog)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    EXPECT_EQ(
        watchdog.state(start),
        HilWatchdogState::HEALTHY);

    EXPECT_TRUE(watchdog.permitsCommand(start + 50ms));
}

TEST(HilCommandWatchdogTests, ExactTimeoutIsAllowed)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    EXPECT_TRUE(watchdog.permitsCommand(start + 100ms));
}

TEST(HilCommandWatchdogTests, ExceedingTimeoutDisablesCommand)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    EXPECT_FALSE(
        watchdog.permitsCommand(start + 101ms));

    EXPECT_EQ(
        watchdog.state(start + 101ms),
        HilWatchdogState::TIMED_OUT);
}

TEST(HilCommandWatchdogTests, RecentCommandRefreshesDeadline)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    ASSERT_TRUE(
        watchdog.observeAcceptedCommand(start + 75ms));

    EXPECT_TRUE(
        watchdog.permitsCommand(start + 150ms));

    EXPECT_FALSE(
        watchdog.permitsCommand(start + 176ms));
}

TEST(HilCommandWatchdogTests, LateCommandCannotRearm)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    EXPECT_FALSE(
        watchdog.observeAcceptedCommand(start + 150ms));

    EXPECT_FALSE(
        watchdog.observeAcceptedCommand(start + 151ms));

    EXPECT_EQ(
        watchdog.state(start + 151ms),
        HilWatchdogState::TIMED_OUT);
}

TEST(HilCommandWatchdogTests, DisconnectIsLatched)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    watchdog.markDisconnected();

    EXPECT_EQ(
        watchdog.state(start + 10ms),
        HilWatchdogState::DISCONNECTED);

    EXPECT_FALSE(
        watchdog.observeAcceptedCommand(start + 20ms));

    EXPECT_FALSE(
        watchdog.permitsCommand(start + 20ms));
}

TEST(HilCommandWatchdogTests, FaultIsLatched)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    watchdog.markFault();

    EXPECT_EQ(
        watchdog.state(start + 10ms),
        HilWatchdogState::FAULTED);

    EXPECT_FALSE(
        watchdog.observeAcceptedCommand(start + 20ms));
}

TEST(HilCommandWatchdogTests, ResetRequiresNewValidCommand)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(watchdog.observeAcceptedCommand(start));

    watchdog.markFault();
    watchdog.reset();

    EXPECT_EQ(
        watchdog.state(start + 1000ms),
        HilWatchdogState::WAITING);

    EXPECT_FALSE(
        watchdog.permitsCommand(start + 1000ms));

    ASSERT_TRUE(
        watchdog.observeAcceptedCommand(start + 1000ms));

    EXPECT_TRUE(
        watchdog.permitsCommand(start + 1001ms));
}

TEST(HilCommandWatchdogTests, BackwardClockIsRejected)
{
    HilCommandWatchdog watchdog(100ms);

    ASSERT_TRUE(
        watchdog.observeAcceptedCommand(start + 100ms));

    EXPECT_FALSE(
        watchdog.observeAcceptedCommand(start + 50ms));

    EXPECT_EQ(
        watchdog.state(start + 100ms),
        HilWatchdogState::TIMED_OUT);
}

TEST(HilCommandWatchdogTests, RejectsInvalidTimeout)
{
    EXPECT_THROW(
        HilCommandWatchdog(0ms),
        std::invalid_argument);

    EXPECT_THROW(
        HilCommandWatchdog(-1ms),
        std::invalid_argument);
}
