#include <gtest/gtest.h>

#include <environment/WindModel.h>

#include <type_traits>

namespace aeroforge::environment::tests
{
	namespace
	{
		class TestWindModel final
			: public WindModel
		{
		public:
			math::Vector3 getWindVelocityN(
				double altitudeMeters,
				double timeSeconds) const override
			{
				(void)altitudeMeters;
				(void)timeSeconds;

				return math::Vector3(
					10.0,
					20.0,
					-5.0);
			}
		};
	}

	TEST(
		WindModelTests,
		IsAbstractInterface)
	{
		EXPECT_TRUE(
			std::is_abstract_v<WindModel>);
	}

	TEST(
		WindModelTests,
		HasVirtualDestructor)
	{
		EXPECT_TRUE(
			std::has_virtual_destructor_v<
			WindModel>);
	}

	TEST(
		WindModelTests,
		SupportsPolymorphicWindQueries)
	{
		TestWindModel concreteModel;

		const WindModel& model =
			concreteModel;

		const math::Vector3 wind =
			model.getWindVelocityN(
				1000.0,
				5.0);

		EXPECT_DOUBLE_EQ(wind.x, 10.0);
		EXPECT_DOUBLE_EQ(wind.y, 20.0);
		EXPECT_DOUBLE_EQ(wind.z, -5.0);
	}
}