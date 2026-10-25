#include <gtest/gtest.h>

#include <aerodynamics/AerodynamicReference.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::aerodynamics::tests
{
	TEST(
		AerodynamicReferenceTests,
		StoresReferenceGeometry)
	{
		const AerodynamicReference reference(
			0.25,
			2.5);

		EXPECT_DOUBLE_EQ(
			reference.getReferenceAreaM2(),
			0.25);

		EXPECT_DOUBLE_EQ(
			reference.getReferenceLengthM(),
			2.5);
	}

	TEST(
		AerodynamicReferenceTests,
		AcceptsSmallPositiveValues)
	{
		const AerodynamicReference reference(
			1e-6,
			1e-3);

		EXPECT_DOUBLE_EQ(
			reference.getReferenceAreaM2(),
			1e-6);

		EXPECT_DOUBLE_EQ(
			reference.getReferenceLengthM(),
			1e-3);
	}

	TEST(
		AerodynamicReferenceTests,
		RejectsZeroArea)
	{
		EXPECT_THROW(
			(AerodynamicReference{
				0.0,
				1.0
				}),
			std::invalid_argument);
	}

	TEST(
		AerodynamicReferenceTests,
		RejectsZeroLength)
	{
		EXPECT_THROW(
			(AerodynamicReference{
				1.0,
				0.0
				}),
			std::invalid_argument);
	}

	TEST(
		AerodynamicReferenceTests,
		RejectsNegativeValues)
	{
		EXPECT_THROW(
			(AerodynamicReference{
				-1.0,
				1.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AerodynamicReference{
				1.0,
				-1.0
				}),
			std::invalid_argument);
	}

	TEST(
		AerodynamicReferenceTests,
		RejectsNonFiniteValues)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		const double infinity =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			(AerodynamicReference{
				nan,
				1.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AerodynamicReference{
				1.0,
				infinity
				}),
			std::invalid_argument);
	}
}