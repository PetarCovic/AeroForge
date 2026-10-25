#include <gtest/gtest.h>

#include <environment/AtmosphereModel.h>

#include <type_traits>

namespace aeroforge::environment::tests
{
	namespace
	{
		class TestAtmosphereModel final
			: public AtmosphereModel
		{
		public:
			AtmosphereState getAtmosphere(
				double altitudeMeters) const override
			{
				(void)altitudeMeters;

				return AtmosphereState(
					288.15,
					101325.0,
					1.225,
					340.294);
			}
		};
	}

	TEST(
		AtmosphereModelTests,
		IsAbstractInterface)
	{
		EXPECT_TRUE(
			std::is_abstract_v<AtmosphereModel>);
	}

	TEST(
		AtmosphereModelTests,
		HasVirtualDestructor)
	{
		EXPECT_TRUE(
			std::has_virtual_destructor_v<
			AtmosphereModel>);
	}

	TEST(
		AtmosphereModelTests,
		SupportsPolymorphicAtmosphereQueries)
	{
		TestAtmosphereModel concreteModel;

		const AtmosphereModel& model =
			concreteModel;

		const AtmosphereState atmosphere =
			model.getAtmosphere(1000.0);

		EXPECT_DOUBLE_EQ(
			atmosphere.getTemperatureK(),
			288.15);

		EXPECT_DOUBLE_EQ(
			atmosphere.getPressurePa(),
			101325.0);
	}
}