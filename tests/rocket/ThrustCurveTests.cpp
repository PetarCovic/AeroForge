#include <gtest/gtest.h>

#include <rocket/ThrustCurve.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace aeroforge::rocket::tests
{
	TEST(
		ThrustCurveTests,
		ReturnsExactPointValues)
	{
		const ThrustCurve curve(
			std::vector<ThrustPoint>{
				{0.0, 0.0},
				{ 1.0, 100.0 },
				{ 2.0, 0.0 }
		});

		EXPECT_DOUBLE_EQ(
			curve.getThrustN(0.0),
			0.0);

		EXPECT_DOUBLE_EQ(
			curve.getThrustN(1.0),
			100.0);

		EXPECT_DOUBLE_EQ(
			curve.getThrustN(2.0),
			0.0);
	}

	TEST(
		ThrustCurveTests,
		LinearlyInterpolatesBetweenPoints)
	{
		const ThrustCurve curve(
			std::vector<ThrustPoint>{
				{0.0, 0.0},
				{ 1.0, 100.0 },
				{ 2.0, 0.0 }
		});

		EXPECT_NEAR(
			curve.getThrustN(0.5),
			50.0,
			1e-12);

		EXPECT_NEAR(
			curve.getThrustN(1.5),
			50.0,
			1e-12);
	}

	TEST(
		ThrustCurveTests,
		ReturnsZeroOutsideCurve)
	{
		const ThrustCurve curve(
			std::vector<ThrustPoint>{
				{1.0, 100.0},
				{ 2.0, 100.0 }
		});

		EXPECT_DOUBLE_EQ(
			curve.getThrustN(0.5),
			0.0);

		EXPECT_DOUBLE_EQ(
			curve.getThrustN(3.0),
			0.0);
	}

	TEST(
		ThrustCurveTests,
		AllowsConstantThrustSegment)
	{
		const ThrustCurve curve(
			std::vector<ThrustPoint>{
				{0.0, 100.0},
				{ 10.0, 100.0 }
		});

		EXPECT_DOUBLE_EQ(
			curve.getThrustN(5.0),
			100.0);
	}

	TEST(
		ThrustCurveTests,
		RejectsTooFewPoints)
	{
		const std::vector<ThrustPoint> points{
			{0.0, 10.0}
		};

		EXPECT_THROW(
			(ThrustCurve(points)),
			std::invalid_argument);
	}

	TEST(
		ThrustCurveTests,
		RejectsNonIncreasingTime)
	{
		const std::vector<ThrustPoint> points{
			{0.0, 10.0},
			{1.0, 20.0},
			{1.0, 30.0}
		};

		EXPECT_THROW(
			(ThrustCurve(points)),
			std::invalid_argument);
	}

	TEST(
		ThrustCurveTests,
		RejectsNegativeThrust)
	{
		const std::vector<ThrustPoint> points{
			{0.0, 10.0},
			{1.0, -1.0}
		};

		EXPECT_THROW(
			(ThrustCurve(points)),
			std::invalid_argument);
	}

	TEST(
		ThrustCurveTests,
		RejectsNonFinitePoints)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		const std::vector<ThrustPoint> points{
			{0.0, 10.0},
			{1.0, nan}
		};

		EXPECT_THROW(
			(ThrustCurve(points)),
			std::invalid_argument);
	}

	TEST(
		ThrustCurveTests,
		RejectsInvalidQueryTime)
	{
		const ThrustCurve curve(
			std::vector<ThrustPoint>{
				{0.0, 10.0},
				{ 1.0, 10.0 }
		});

		EXPECT_THROW(
			curve.getThrustN(-1.0),
			std::invalid_argument);

		EXPECT_THROW(
			curve.getThrustN(
				std::numeric_limits<double>::
				infinity()),
			std::invalid_argument);
	}
}