#include <gtest/gtest.h>

#include <aerodynamics/AerodynamicCoefficients.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::aerodynamics::tests
{
	TEST(
		AerodynamicCoefficientsTests,
		StoresCoefficients)
	{
		const AerodynamicCoefficients coefficients(
			0.45,
			0.8,
			-0.15);

		EXPECT_DOUBLE_EQ(
			coefficients.getDragCoefficient(),
			0.45);

		EXPECT_DOUBLE_EQ(
			coefficients.getLiftCoefficient(),
			0.8);

		EXPECT_DOUBLE_EQ(
			coefficients.getSideForceCoefficient(),
			-0.15);
	}

	TEST(
		AerodynamicCoefficientsTests,
		AllowsZeroCoefficients)
	{
		const AerodynamicCoefficients coefficients(
			0.0,
			0.0,
			0.0);

		EXPECT_DOUBLE_EQ(
			coefficients.getDragCoefficient(),
			0.0);

		EXPECT_DOUBLE_EQ(
			coefficients.getLiftCoefficient(),
			0.0);

		EXPECT_DOUBLE_EQ(
			coefficients.getSideForceCoefficient(),
			0.0);
	}

	TEST(
		AerodynamicCoefficientsTests,
		AllowsNegativeLiftAndSideForce)
	{
		const AerodynamicCoefficients coefficients(
			0.5,
			-1.2,
			-0.3);

		EXPECT_DOUBLE_EQ(
			coefficients.getLiftCoefficient(),
			-1.2);

		EXPECT_DOUBLE_EQ(
			coefficients.getSideForceCoefficient(),
			-0.3);
	}

	TEST(
		AerodynamicCoefficientsTests,
		RejectsNegativeDragCoefficient)
	{
		EXPECT_THROW(
			(AerodynamicCoefficients{
				-0.01,
				0.0,
				0.0
				}),
			std::invalid_argument);
	}

	TEST(
		AerodynamicCoefficientsTests,
		RejectsNonFiniteCoefficients)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		const double infinity =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			(AerodynamicCoefficients{
				nan,
				0.0,
				0.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AerodynamicCoefficients{
				0.5,
				infinity,
				0.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AerodynamicCoefficients{
				0.5,
				0.0,
				nan
				}),
			std::invalid_argument);
	}
}