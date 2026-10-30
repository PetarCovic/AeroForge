#include <gtest/gtest.h>

#include <hil/HilHardwareRun.h>
#include <hil/HilTelemetryStore.h>

#include <limits>
#include <stdexcept>
#include <stop_token>

using namespace aeroforge::hil;

TEST(HilHardwareRunTests, DefaultConfigurationValid)
{
    EXPECT_NO_THROW(validateHilHardwareRunConfig(HilHardwareRunConfig{}));
}

TEST(HilHardwareRunTests, RejectsInvalidSettings)
{
    HilHardwareRunConfig config{};
    config.portName.clear();
    EXPECT_THROW(validateHilHardwareRunConfig(config), std::invalid_argument);
    config = {};
    config.baudRate = 0;
    EXPECT_THROW(validateHilHardwareRunConfig(config), std::invalid_argument);
    config = {};
    config.stepS = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(validateHilHardwareRunConfig(config), std::invalid_argument);
    config = {};
    config.steps = 0;
    EXPECT_THROW(validateHilHardwareRunConfig(config), std::invalid_argument);
    config = {};
    config.commandTimeout = std::chrono::milliseconds(0);
    EXPECT_THROW(validateHilHardwareRunConfig(config), std::invalid_argument);
}

#ifdef _WIN32
TEST(HilHardwareRunTests, PriorCancellationDoesNotOpenSerialPort)
{
    HilTelemetryStore telemetry;
    std::stop_source stop;
    stop.request_stop();
    HilHardwareRunConfig config{};
    config.portName = L"COM9999";
    config.steps = 1;
    const auto result = runHilHardwareClosedLoop(
        telemetry, nullptr, stop.get_token(), config);
    EXPECT_TRUE(result.cancelled);
    EXPECT_EQ(result.completedSteps, 0u);
    EXPECT_FALSE(result.finalHealth.running);
}
#endif
