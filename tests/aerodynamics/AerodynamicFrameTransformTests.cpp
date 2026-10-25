#include <gtest/gtest.h>

#include <aerodynamics/AerodynamicFrameTransform.h>

#include <math/Vector3.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace aeroforge::aerodynamics::tests
{
	TEST(
		AerodynamicFrameTransformTests,
		ZeroAnglesLeaveVectorUnchanged)
	{
		const math::Vector3 vectorWind(
			1.0,
			2.0,
			3.0);

		const math::Vector3 vectorBody =
			transformWindToBody(
				vectorWind,
				0.0,
				0.0);

		EXPECT_NEAR(vectorBody.x, 1.0, 1e-12);
		EXPECT_NEAR(vectorBody.y, 2.0, 1e-12);
		EXPECT_NEAR(vectorBody.z, 3.0, 1e-12);
	}

	TEST(
		AerodynamicFrameTransformTests,
		NinetyDegreeAlphaRotatesWindXIntoBodyZ)
	{
		const double halfPi =
			std::numbers::pi_v<double> / 2.0;

		const math::Vector3 vectorBody =
			transformWindToBody(
				math::Vector3(
					1.0,
					0.0,
					0.0),
				halfPi,
				0.0);

		EXPECT_NEAR(vectorBody.x, 0.0, 1e-12);
		EXPECT_NEAR(vectorBody.y, 0.0, 1e-12);
		EXPECT_NEAR(vectorBody.z, 1.0, 1e-12);
	}

	TEST(
		AerodynamicFrameTransformTests,
		NinetyDegreeBetaRotatesWindXIntoBodyY)
	{
		const double halfPi =
			std::numbers::pi_v<double> / 2.0;

		const math::Vector3 vectorBody =
			transformWindToBody(
				math::Vector3(
					1.0,
					0.0,
					0.0),
				0.0,
				halfPi);

		EXPECT_NEAR(vectorBody.x, 0.0, 1e-12);
		EXPECT_NEAR(vectorBody.y, 1.0, 1e-12);
		EXPECT_NEAR(vectorBody.z, 0.0, 1e-12);
	}

	TEST(
		AerodynamicFrameTransformTests,
		PreservesVectorMagnitude)
	{
		const math::Vector3 vectorWind(
			3.0,
			4.0,
			12.0);

		const math::Vector3 vectorBody =
			transformWindToBody(
				vectorWind,
				0.4,
				-0.2);

		const double windMagnitude =
			std::sqrt(
				vectorWind.x * vectorWind.x
				+ vectorWind.y * vectorWind.y
				+ vectorWind.z * vectorWind.z);

		const double bodyMagnitude =
			std::sqrt(
				vectorBody.x * vectorBody.x
				+ vectorBody.y * vectorBody.y
				+ vectorBody.z * vectorBody.z);

		EXPECT_NEAR(
			bodyMagnitude,
			windMagnitude,
			1e-12);
	}

	TEST(
		AerodynamicFrameTransformTests,
		RejectsNonFiniteInput)
	{
		const math::Vector3 vectorWind(
			std::numeric_limits<double>::infinity(),
			0.0,
			0.0);

		EXPECT_THROW(
			transformWindToBody(
				vectorWind,
				0.0,
				0.0),
			std::invalid_argument);
	}

	TEST(
		AerodynamicFrameTransformTests,
		RejectsNonFiniteAngles)
	{
		const math::Vector3 vectorWind(
			1.0,
			0.0,
			0.0);

		EXPECT_THROW(
			transformWindToBody(
				vectorWind,
				std::numeric_limits<double>::
				quiet_NaN(),
				0.0),
			std::invalid_argument);
	}
}