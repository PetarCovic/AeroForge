#include <gtest/gtest.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyState.h>
#include <dynamics/SixDoFDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

namespace aeroforge::dynamics::tests
{
	namespace
	{
		InertiaProperties makeSphericalInertia()
		{
			return InertiaProperties(
				math::Matrix(
					3,
					3,
					{
						2.0, 0.0, 0.0,
						0.0, 2.0, 0.0,
						0.0, 0.0, 2.0
					}));
		}
	}

	TEST(
		SixDoFDynamicsModelTests,
		ReturnsThirteenElementDerivative)
	{
		ForceAccumulator forces;
		MomentAccumulator moments;
		MassProperties mass(10.0);

		const InertiaProperties inertia =
			makeSphericalInertia();

		const SixDoFDynamicsModel model(
			forces,
			moments,
			mass,
			inertia);

		const RigidBodyState rigidBodyState(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0));

		const simulation::StateDerivative derivative =
			model.evaluate(
				0.0,
				rigidBodyState.toStateVector());

		EXPECT_EQ(
			derivative.size(),
			13);
	}

	TEST(
		SixDoFDynamicsModelTests,
		DerivativeLayoutIsCorrect)
	{
		ForceAccumulator forces;

		forces.addForce(
			math::Vector3(
				10.0,
				20.0,
				30.0));

		MomentAccumulator moments;

		MassProperties mass(
			2.0);

		const InertiaProperties inertia =
			makeSphericalInertia();

		const SixDoFDynamicsModel model(
			forces,
			moments,
			mass,
			inertia);

		const RigidBodyState rigidBodyState(
			math::Vector3(
				100.0,
				200.0,
				300.0),
			math::Vector3(
				1.0,
				2.0,
				3.0),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.2,
				0.4,
				0.6));

		const simulation::StateDerivative derivative =
			model.evaluate(
				0.0,
				rigidBodyState.toStateVector());

		// Position derivative = velocity.
		EXPECT_DOUBLE_EQ(derivative[0], 1.0);
		EXPECT_DOUBLE_EQ(derivative[1], 2.0);
		EXPECT_DOUBLE_EQ(derivative[2], 3.0);

		// Linear acceleration = F / m.
		EXPECT_DOUBLE_EQ(derivative[3], 5.0);
		EXPECT_DOUBLE_EQ(derivative[4], 10.0);
		EXPECT_DOUBLE_EQ(derivative[5], 15.0);

		// qdot = 0.5 * q * [0, omega].
		EXPECT_DOUBLE_EQ(derivative[6], 0.0);
		EXPECT_DOUBLE_EQ(derivative[7], 0.1);
		EXPECT_DOUBLE_EQ(derivative[8], 0.2);
		EXPECT_DOUBLE_EQ(derivative[9], 0.3);

		// Spherical inertia + zero moment means
		// constant body angular velocity.
		EXPECT_NEAR(derivative[10], 0.0, 1e-12);
		EXPECT_NEAR(derivative[11], 0.0, 1e-12);
		EXPECT_NEAR(derivative[12], 0.0, 1e-12);
	}

	TEST(
		SixDoFDynamicsModelTests,
		AppliedMomentProducesAngularAcceleration)
	{
		ForceAccumulator forces;

		MomentAccumulator moments;

		moments.addMoment(
			math::Vector3(
				2.0,
				4.0,
				6.0));

		MassProperties mass(
			10.0);

		const InertiaProperties inertia =
			makeSphericalInertia();

		const SixDoFDynamicsModel model(
			forces,
			moments,
			mass,
			inertia);

		const RigidBodyState rigidBodyState(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0));

		const simulation::StateDerivative derivative =
			model.evaluate(
				0.0,
				rigidBodyState.toStateVector());

		EXPECT_NEAR(derivative[10], 1.0, 1e-12);
		EXPECT_NEAR(derivative[11], 2.0, 1e-12);
		EXPECT_NEAR(derivative[12], 3.0, 1e-12);
	}

	TEST(
		SixDoFDynamicsModelTests,
		UsesUpdatedForceAndMomentAccumulators)
	{
		ForceAccumulator forces;
		MomentAccumulator moments;

		MassProperties mass(
			2.0);

		const InertiaProperties inertia =
			makeSphericalInertia();

		const SixDoFDynamicsModel model(
			forces,
			moments,
			mass,
			inertia);

		const RigidBodyState rigidBodyState(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0));

		forces.addForce(
			math::Vector3(
				10.0,
				0.0,
				0.0));

		moments.addMoment(
			math::Vector3(
				2.0,
				0.0,
				0.0));

		const simulation::StateDerivative first =
			model.evaluate(
				0.0,
				rigidBodyState.toStateVector());

		EXPECT_DOUBLE_EQ(first[3], 5.0);
		EXPECT_DOUBLE_EQ(first[10], 1.0);

		forces.clear();
		moments.clear();

		const simulation::StateDerivative second =
			model.evaluate(
				0.0,
				rigidBodyState.toStateVector());

		EXPECT_DOUBLE_EQ(second[3], 0.0);
		EXPECT_DOUBLE_EQ(second[10], 0.0);
	}
}