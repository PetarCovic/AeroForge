#include <gtest/gtest.h>

#include <telemetry/GncAnalyzer.h>
#include <telemetry/TelemetrySample.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace aeroforge::telemetry
{
	namespace
	{
		TelemetrySample makeGncSample(
			double timeS,
			const math::Vector3& errorB,
			const math::Vector3& commandB,
			bool saturated)
		{
			return TelemetrySample{
				timeS,

				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),

				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),

				errorB,
				commandB,
				commandB,

				0.0,
				0.0,
				0.0,

				saturated
			};
		}
	}


	TEST(GncAnalyzerTests, RejectsEmptyTelemetry)
	{
		EXPECT_THROW(
			GncAnalyzer::analyze(
				{},
				0.01
			),
			std::invalid_argument
		);
	}


	TEST(GncAnalyzerTests, RejectsNonPositiveSettlingThreshold)
	{
		EXPECT_THROW(
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(),
						math::Vector3(),
						false
					)
				},
				0.0
			),
			std::invalid_argument
		);
	}


	TEST(GncAnalyzerTests, ComputesPeakAttitudeError)
	{
		auto metrics =
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(0.1, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						1.0,
						math::Vector3(0.0, 0.3, 0.4),
						math::Vector3(),
						false
					)
				},
				0.01
			);

		EXPECT_DOUBLE_EQ(
			metrics.peakAttitudeErrorRad,
			0.5
		);
	}


	TEST(GncAnalyzerTests, ComputesRmsAttitudeError)
	{
		auto metrics =
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(
							3.0,
							4.0,
							0.0
						),
						math::Vector3(),
						false
					),

					makeGncSample(
						1.0,
						math::Vector3(),
						math::Vector3(),
						false
					)
				},
				0.01
			);

		EXPECT_NEAR(
			metrics.rmsAttitudeErrorRad,
			std::sqrt(12.5),
			1e-12
		);
	}


	TEST(GncAnalyzerTests, ComputesMeanCommandedMomentMagnitude)
	{
		auto metrics =
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(),
						math::Vector3(
							3.0,
							4.0,
							0.0
						),
						false
					),

					makeGncSample(
						1.0,
						math::Vector3(),
						math::Vector3(
							0.0,
							0.0,
							5.0
						),
						false
					)
				},
				0.01
			);

		EXPECT_DOUBLE_EQ(
			metrics.meanCommandedMomentNm,
			5.0
		);

		EXPECT_DOUBLE_EQ(
			metrics.maxCommandedMomentNm,
			5.0
		);
	}


	TEST(GncAnalyzerTests, ComputesSaturationFraction)
	{
		auto metrics =
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(),
						math::Vector3(),
						true
					),

					makeGncSample(
						1.0,
						math::Vector3(),
						math::Vector3(),
						false
					),

					makeGncSample(
						2.0,
						math::Vector3(),
						math::Vector3(),
						true
					),

					makeGncSample(
						3.0,
						math::Vector3(),
						math::Vector3(),
						false
					)
				},
				0.01
			);

		EXPECT_DOUBLE_EQ(
			metrics.saturationFraction,
			0.5
		);
	}


	TEST(GncAnalyzerTests, ComputesSettlingTime)
	{
		auto metrics =
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(0.5, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						1.0,
						math::Vector3(0.2, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						2.0,
						math::Vector3(0.04, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						3.0,
						math::Vector3(0.02, 0.0, 0.0),
						math::Vector3(),
						false
					)
				},
				0.05
			);

		ASSERT_TRUE(
			metrics.settlingTimeS.has_value()
		);

		EXPECT_DOUBLE_EQ(
			*metrics.settlingTimeS,
			2.0
		);
	}


	TEST(GncAnalyzerTests, IgnoresTemporaryThresholdCrossing)
	{
		auto metrics =
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(0.5, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						1.0,
						math::Vector3(0.04, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						2.0,
						math::Vector3(0.10, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						3.0,
						math::Vector3(0.03, 0.0, 0.0),
						math::Vector3(),
						false
					)
				},
				0.05
			);

		ASSERT_TRUE(
			metrics.settlingTimeS.has_value()
		);

		EXPECT_DOUBLE_EQ(
			*metrics.settlingTimeS,
			3.0
		);
	}


	TEST(GncAnalyzerTests, ReportsNoSettlingWhenErrorNeverSettles)
	{
		auto metrics =
			GncAnalyzer::analyze(
				{
					makeGncSample(
						0.0,
						math::Vector3(0.5, 0.0, 0.0),
						math::Vector3(),
						false
					),

					makeGncSample(
						1.0,
						math::Vector3(0.3, 0.0, 0.0),
						math::Vector3(),
						false
					)
				},
				0.05
			);

		EXPECT_FALSE(
			metrics.settlingTimeS.has_value()
		);
	}
}