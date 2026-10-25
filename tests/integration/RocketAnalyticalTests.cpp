#include <gtest/gtest.h>

#include <rocket/PropellantTank.h>
#include <rocket/RocketDynamicsModel.h>
#include <rocket/RocketEngine.h>
#include <rocket/RocketMassProperties.h>
#include <rocket/ThrustCurve.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyQuaternionNormalization.h>
#include <dynamics/RigidBodyState.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <vector>

namespace aeroforge::integration::tests
{
	TEST(
		RocketAnalyticalTests,
		ConstantThrustMatchesConstantAccelerationSolution)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		rocket::PropellantTank tank(
			1.0,
			1.0);

		const rocket::RocketMassProperties massProperties(
			9.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix::diagonal(
				2.0,
				3),
			math::Vector3(0.0, 0.0, 0.0));

		const rocket::RocketEngine engine(
			rocket::ThrustCurve(
				std::vector<rocket::ThrustPoint>{
					{0.0, 20.0},
					{ 10.0, 20.0 }
		}),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const rocket::RocketDynamicsModel model(
			forces,
			moments,
			tank,
			massProperties,
			engine);

		const simulation::RK4Integrator integrator;

		simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		constexpr double dt =
			0.01;

		for (int step = 0;
			step < 100;
			++step)
		{
			state =
				integrator.step(
					model,
					step * dt,
					state,
					dt);

			dynamics::
				normalizeRigidBodyQuaternion(
					state);
		}

		EXPECT_NEAR(
			state[0],
			1.0,
			1e-9);

		EXPECT_NEAR(
			state[3],
			2.0,
			1e-9);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantMassKg(),
			1.0);
	}
}