#include <gtest/gtest.h>

#include <aerodynamics/AerodynamicModel.h>

#include <type_traits>

namespace aeroforge::aerodynamics::tests
{
	namespace
	{
		class TestAerodynamicModel final
			: public AerodynamicModel
		{
		public:
			math::Vector3 computeForceBody(
				const environment::AtmosphereState&
				atmosphere,
				const math::Vector3&
				relativeAirVelocityB)
				const override
			{
				(void)atmosphere;
				(void)relativeAirVelocityB;

				return math::Vector3(
					1.0,
					2.0,
					3.0);
			}
		};
	}

	TEST(
		AerodynamicModelTests,
		IsAbstractInterface)
	{
		EXPECT_TRUE(
			std::is_abstract_v<
			AerodynamicModel>);
	}

	TEST(
		AerodynamicModelTests,
		HasVirtualDestructor)
	{
		EXPECT_TRUE(
			std::has_virtual_destructor_v<
			AerodynamicModel>);
	}

	TEST(
		AerodynamicModelTests,
		SupportsPolymorphicForceCalculation)
	{
		TestAerodynamicModel concrete;

		const AerodynamicModel& model =
			concrete;

		const environment::AtmosphereState atmosphere(
			288.15,
			101325.0,
			1.225,
			340.0);

		const math::Vector3 force =
			model.computeForceBody(
				atmosphere,
				math::Vector3(
					100.0,
					0.0,
					0.0));

		EXPECT_DOUBLE_EQ(force.x, 1.0);
		EXPECT_DOUBLE_EQ(force.y, 2.0);
		EXPECT_DOUBLE_EQ(force.z, 3.0);
	}
}