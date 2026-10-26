#include <visualization/SixDoFLiveSimulation.h>
#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/VisualizationSession.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <stop_token>
#include <stdexcept>

namespace aeroforge::visualization
{
    TEST(SixDoFLiveSimulationTests, ProducesIntegratedNonconstant6DoFStates)
    {
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        const auto result = runSixDoFLiveSimulation(*publisher, {}, 0.01, 1000, false);

        EXPECT_FALSE(result.cancelled);
        EXPECT_EQ(result.publicationAttempts, 1001u);
        EXPECT_NEAR(result.lastSimulationTimeS, 10.0, 1e-9);

        const auto last = publisher->getLatestSnapshot();
        ASSERT_TRUE(last.has_value());
        EXPECT_NEAR(last->simulationTimeS, 10.0, 1e-9);
        EXPECT_LT(last->truthPositionN.z, 0.0);
        EXPECT_NEAR(last->truthAttitudeNB.squaredNorm(), 1.0, 1e-10);
        EXPECT_GT(std::abs(last->truthAttitudeNB.w - std::sqrt(0.5)), 0.1);
        EXPECT_GT(std::abs(last->truthAngularVelocityB.y), 0.001);
        // This demo does not run a navigation estimator.
        EXPECT_DOUBLE_EQ(last->estimatedPositionN.z, last->truthPositionN.z);

        VisualizationSession session;
        session.attachLive(publisher);
        session.update(0.0);
        ASSERT_NE(session.current(), nullptr);
        EXPECT_NEAR(session.currentTimeS(), 10.0, 1e-9);
        EXPECT_EQ(session.mode(), VisualizationSession::Mode::Live);
    }

    TEST(SixDoFLiveSimulationTests, HonorsCancellationBeforeFirstStep)
    {
        LiveTelemetryPublisher publisher;
        std::stop_source source;
        source.request_stop();
        const auto result = runSixDoFLiveSimulation(
            publisher, source.get_token(), 0.01, 1000, false);
        EXPECT_TRUE(result.cancelled);
        EXPECT_EQ(result.publicationAttempts, 0u);
        EXPECT_FALSE(publisher.hasSnapshot());
    }

    TEST(SixDoFLiveSimulationTests, RejectsInvalidTimeSteps)
    {
        LiveTelemetryPublisher publisher;
        EXPECT_THROW(runSixDoFLiveSimulation(publisher, {}, 0.0, 1, false), std::invalid_argument);
        EXPECT_THROW(runSixDoFLiveSimulation(publisher, {}, -0.01, 1, false), std::invalid_argument);
        EXPECT_THROW(runSixDoFLiveSimulation(publisher, {},
            std::numeric_limits<double>::quiet_NaN(), 1, false), std::invalid_argument);
    }
}
