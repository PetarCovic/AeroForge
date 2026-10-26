#include <visualization/VisualizationInterpolator.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aeroforge::visualization
{
	namespace
	{
		void expectVectorNear(
			const math::Vector3& actual,
			const math::Vector3& expected,
			double tolerance = 1e-9)
		{
			EXPECT_NEAR(
				actual.x,
				expected.x,
				tolerance);

			EXPECT_NEAR(
				actual.y,
				expected.y,
				tolerance);

			EXPECT_NEAR(
				actual.z,
				expected.z,
				tolerance);
		}

		void expectSameRotation(
			const math::Quaternion& actual,
			const math::Quaternion& expected,
			double tolerance = 1e-9)
		{
			EXPECT_TRUE(
				actual.isSameRotation(
					expected,
					tolerance));
		}

		VisualizationSnapshot makeSnapshot(
			double timeS,
			double baseValue,
			const math::Quaternion& truthAttitude,
			const math::Quaternion& estimatedAttitude)
		{
			return VisualizationSnapshot{
				timeS,

				math::Vector3(
					baseValue + 1.0,
					baseValue + 2.0,
					baseValue + 3.0),

				math::Vector3(
					baseValue + 4.0,
					baseValue + 5.0,
					baseValue + 6.0),

				truthAttitude,

				math::Vector3(
					baseValue + 7.0,
					baseValue + 8.0,
					baseValue + 9.0),

				math::Vector3(
					baseValue + 10.0,
					baseValue + 11.0,
					baseValue + 12.0),

				math::Vector3(
					baseValue + 13.0,
					baseValue + 14.0,
					baseValue + 15.0),

				estimatedAttitude,

				math::Vector3(
					baseValue + 16.0,
					baseValue + 17.0,
					baseValue + 18.0),

				math::Vector3(
					baseValue + 19.0,
					baseValue + 20.0,
					baseValue + 21.0),

				math::Vector3(
					baseValue + 22.0,
					baseValue + 23.0,
					baseValue + 24.0),

				math::Vector3(
					baseValue + 25.0,
					baseValue + 26.0,
					baseValue + 27.0)
			};
		}
	}

	TEST(
		VisualizationInterpolatorTests,
		ReturnsFirstSnapshotBeforeRange)
	{
		const auto first =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				20.0,
				10.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				5.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			first.simulationTimeS);

		expectVectorNear(
			result.truthPositionN,
			first.truthPositionN);
	}

	TEST(
		VisualizationInterpolatorTests,
		ReturnsSecondSnapshotAfterRange)
	{
		const auto first =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				20.0,
				10.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				25.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			second.simulationTimeS);

		expectVectorNear(
			result.truthPositionN,
			second.truthPositionN);
	}

	TEST(
		VisualizationInterpolatorTests,
		ReturnsFirstSnapshotAtFirstTimestamp)
	{
		const auto first =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				20.0,
				10.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				10.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			10.0);

		expectVectorNear(
			result.truthVelocityN,
			first.truthVelocityN);
	}

	TEST(
		VisualizationInterpolatorTests,
		ReturnsSecondSnapshotAtSecondTimestamp)
	{
		const auto first =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				20.0,
				10.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				20.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			20.0);

		expectVectorNear(
			result.truthVelocityN,
			second.truthVelocityN);
	}

	TEST(
		VisualizationInterpolatorTests,
		InterpolatesAllLinearFieldsAtMidpoint)
	{
		const auto first =
			makeSnapshot(
				0.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				10.0,
				10.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				5.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			5.0);

		expectVectorNear(
			result.truthPositionN,
			math::Vector3(
				6.0,
				7.0,
				8.0));

		expectVectorNear(
			result.truthVelocityN,
			math::Vector3(
				9.0,
				10.0,
				11.0));

		expectVectorNear(
			result.truthAngularVelocityB,
			math::Vector3(
				12.0,
				13.0,
				14.0));

		expectVectorNear(
			result.estimatedPositionN,
			math::Vector3(
				15.0,
				16.0,
				17.0));

		expectVectorNear(
			result.estimatedVelocityN,
			math::Vector3(
				18.0,
				19.0,
				20.0));

		expectVectorNear(
			result.estimatedAngularVelocityB,
			math::Vector3(
				21.0,
				22.0,
				23.0));

		expectVectorNear(
			result.attitudeErrorB,
			math::Vector3(
				24.0,
				25.0,
				26.0));

		expectVectorNear(
			result.commandedMomentB,
			math::Vector3(
				27.0,
				28.0,
				29.0));

		expectVectorNear(
			result.actualMomentB,
			math::Vector3(
				30.0,
				31.0,
				32.0));
	}

	TEST(
		VisualizationInterpolatorTests,
		InterpolatesTruthAttitudeUsingSlerp)
	{
		const double pi =
			std::acos(-1.0);

		const auto first =
			makeSnapshot(
				0.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::fromRotationZ(
					pi),
				math::Quaternion::identity());

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				5.0);

		const auto expected =
			math::Quaternion::fromRotationZ(
				pi * 0.5);

		expectSameRotation(
			result.truthAttitudeNB,
			expected);
	}

	TEST(
		VisualizationInterpolatorTests,
		InterpolatesEstimatedAttitudeUsingSlerp)
	{
		const double pi =
			std::acos(-1.0);

		const auto first =
			makeSnapshot(
				0.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::fromRotationX(
					pi));

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				5.0);

		const auto expected =
			math::Quaternion::fromRotationX(
				pi * 0.5);

		expectSameRotation(
			result.estimatedAttitudeNB,
			expected);
	}

	TEST(
		VisualizationInterpolatorTests,
		QuaternionInterpolationRemainsNormalized)
	{
		const auto first =
			makeSnapshot(
				0.0,
				0.0,
				math::Quaternion::fromRotationX(
					0.2),
				math::Quaternion::fromRotationY(
					0.3));

		const auto second =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::fromRotationX(
					1.2),
				math::Quaternion::fromRotationY(
					1.3));

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				4.0);

		EXPECT_TRUE(
			result.truthAttitudeNB.isNormalized(
				1e-9));

		EXPECT_TRUE(
			result.estimatedAttitudeNB.isNormalized(
				1e-9));
	}

	TEST(
		VisualizationInterpolatorTests,
		QuaternionInterpolationUsesShortestPath)
	{
		const double pi =
			std::acos(-1.0);

		const math::Quaternion start =
			math::Quaternion::identity();

		math::Quaternion end =
			math::Quaternion::fromRotationZ(
				pi * 0.5);

		end *= -1.0;

		const auto first =
			makeSnapshot(
				0.0,
				0.0,
				start,
				start);

		const auto second =
			makeSnapshot(
				10.0,
				0.0,
				end,
				end);

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				5.0);

		const auto expected =
			math::Quaternion::fromRotationZ(
				pi * 0.25);

		expectSameRotation(
			result.truthAttitudeNB,
			expected);
	}

	TEST(
		VisualizationInterpolatorTests,
		EqualSnapshotTimesReturnFirstSnapshot)
	{
		const auto first =
			makeSnapshot(
				10.0,
				1.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				10.0,
				100.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto result =
			VisualizationInterpolator::interpolate(
				first,
				second,
				10.0);

		expectVectorNear(
			result.truthPositionN,
			first.truthPositionN);
	}

	TEST(
		VisualizationInterpolatorTests,
		RejectsReversedSnapshotTimes)
	{
		const auto first =
			makeSnapshot(
				20.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		EXPECT_THROW(
			VisualizationInterpolator::interpolate(
				first,
				second,
				15.0),
			std::invalid_argument);
	}

	TEST(
		VisualizationInterpolatorTests,
		RejectsNonFiniteTargetTime)
	{
		const auto first =
			makeSnapshot(
				0.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		const auto second =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		EXPECT_THROW(
			VisualizationInterpolator::interpolate(
				first,
				second,
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);

		EXPECT_THROW(
			VisualizationInterpolator::interpolate(
				first,
				second,
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);
	}

	TEST(
		VisualizationInterpolatorTests,
		RejectsNonFiniteSnapshotTimes)
	{
		auto first =
			makeSnapshot(
				0.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		auto second =
			makeSnapshot(
				10.0,
				0.0,
				math::Quaternion::identity(),
				math::Quaternion::identity());

		first.simulationTimeS =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			VisualizationInterpolator::interpolate(
				first,
				second,
				5.0),
			std::invalid_argument);

		first.simulationTimeS = 0.0;

		second.simulationTimeS =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			VisualizationInterpolator::interpolate(
				first,
				second,
				5.0),
			std::invalid_argument);
	}
}