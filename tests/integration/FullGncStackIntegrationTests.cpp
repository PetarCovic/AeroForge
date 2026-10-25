#include <gtest/gtest.h>

#include <actuators/BodyMomentActuator.h>

#include <gnc/AttitudeError.h>
#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeSetpoint.h>
#include <gnc/HoldAttitudeGuidance.h>

#include <navigation/GpsCorrector.h>
#include <navigation/InertialNavigator.h>
#include <navigation/NavigationState.h>

#include <sensors/AccelerometerModel.h>
#include <sensors/GpsModel.h>
#include <sensors/GyroscopeModel.h>
#include <sensors/ImuModel.h>
#include <sensors/SensorSchedule.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyQuaternionNormalization.h>
#include <dynamics/RigidBodyState.h>
#include <dynamics/SixDoFDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <cmath>

namespace aeroforge::integration::tests
{
	TEST(
		FullGncStackIntegrationTests,
		SensorsNavigationGuidanceControlAndActuationConvergeAttitude)
	{
		//
		// TRUTH DYNAMICS
		//

		dynamics::ForceAccumulator forceAccumulator;
		dynamics::MomentAccumulator momentAccumulator;

		const dynamics::MassProperties massProperties(
			10.0);

		const dynamics::InertiaProperties inertiaProperties(
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 2.0, 0.0,
					0.0, 0.0, 2.0
				}));

		const dynamics::SixDoFDynamicsModel dynamicsModel(
			forceAccumulator,
			momentAccumulator,
			massProperties,
			inertiaProperties);

		const simulation::RK4Integrator integrator;

		//
		// PERFECT SENSORS
		//

		sensors::AccelerometerModel accelerometer(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				100.0,
				100.0,
				100.0),
			1001);

		sensors::GyroscopeModel gyroscope(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				20.0,
				20.0,
				20.0),
			1002);

		sensors::ImuModel imu(
			accelerometer,
			gyroscope);

		sensors::GpsModel gps(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0),
			2001);

		sensors::SensorSchedule gpsSchedule(
			10.0,
			0.0);

		//
		// NAVIGATION
		//

		const navigation::InertialNavigator inertialNavigator;

		const navigation::GpsCorrector gpsCorrector(
			1.0,
			1.0);

		//
		// GUIDANCE
		//

		const gnc::AttitudeSetpoint desiredAttitude(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0));

		const gnc::HoldAttitudeGuidance guidance(
			desiredAttitude);

		//
		// CONTROLLER
		//

		const gnc::AttitudePdController controller(
			math::Vector3(
				8.0,
				8.0,
				8.0),
			math::Vector3(
				5.0,
				5.0,
				5.0),
			math::Vector3(
				20.0,
				20.0,
				20.0));

		//
		// ACTUATOR
		//

		actuators::BodyMomentActuator actuator(
			math::Vector3(
				20.0,
				20.0,
				20.0),
			math::Vector3(
				200.0,
				200.0,
				200.0));

		//
		// INITIAL TRUTH STATE
		//

		const double initialYawRad =
			20.0
			* 3.14159265358979323846
			/ 180.0;

		simulation::State truthState =
			dynamics::RigidBodyState(
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Quaternion(
					std::cos(
						initialYawRad / 2.0),
					0.0,
					0.0,
					std::sin(
						initialYawRad / 2.0)),
				math::Vector3(
					0.0,
					0.0,
					0.0))
			.toStateVector();

		//
		// INITIAL NAVIGATION ESTIMATE
		//

		const dynamics::RigidBodyState initialTruth(
			truthState);

		navigation::NavigationState navigationState(
			initialTruth.getPositionN(),
			initialTruth.getVelocityN(),
			initialTruth.getAttitudeNB(),
			initialTruth.getAngularVelocityB());

		//
		// SIMULATION
		//

		constexpr double dt =
			0.005;

		constexpr int steps =
			1600;

		for (int step = 0;
			step < steps;
			++step)
		{
			const double time =
				step * dt;

			//
			// READ CURRENT TRUTH
			//

			const dynamics::RigidBodyState truth(
				truthState);

			//
			// SENSOR GENERATION
			//
			// This test intentionally uses zero gravity
			// and zero translational acceleration so the
			// true specific force is zero.
			//

			const sensors::ImuMeasurement imuMeasurement =
				imu.sample(
					time,
					math::Vector3(
						0.0,
						0.0,
						0.0),
					truth.getAngularVelocityB());

			//
			// INERTIAL NAVIGATION PROPAGATION
			//

			navigationState =
				inertialNavigator.propagate(
					navigationState,
					imuMeasurement,
					math::Vector3(
						0.0,
						0.0,
						0.0),
					dt);

			//
			// GPS UPDATE
			//

			if (gpsSchedule.shouldSample(time))
			{
				const sensors::GpsMeasurement gpsMeasurement =
					gps.sample(
						time,
						truth.getPositionN(),
						truth.getVelocityN());

				navigationState =
					gpsCorrector.correct(
						navigationState,
						gpsMeasurement);
			}

			//
			// GUIDANCE
			//

			const gnc::AttitudeSetpoint setpoint =
				guidance.getSetpoint(
					time);

			//
			// CONTROL
			//

			const math::Vector3 commandedMomentB =
				controller.computeMomentCommandB(
					setpoint,
					navigationState);

			//
			// ACTUATOR DYNAMICS
			//

			const math::Vector3 actualMomentB =
				actuator.update(
					commandedMomentB,
					dt);

			//
			// APPLY ACTUATOR OUTPUT
			//

			momentAccumulator.clear();

			momentAccumulator.addMoment(
				actualMomentB);

			//
			// PROPAGATE TRUE 6DOF VEHICLE
			//

			truthState =
				integrator.step(
					dynamicsModel,
					time,
					truthState,
					dt);

			dynamics::
				normalizeRigidBodyQuaternion(
					truthState);
		}

		//
		// FINAL TRUTH
		//

		const dynamics::RigidBodyState finalTruth(
			truthState);

		const math::Vector3 finalTruthError =
			gnc::computeAttitudeErrorVectorB(
				desiredAttitude.getAttitudeNB(),
				finalTruth.getAttitudeNB());

		//
		// FINAL NAVIGATION ERROR
		//

		const math::Vector3 finalNavigationError =
			gnc::computeAttitudeErrorVectorB(
				desiredAttitude.getAttitudeNB(),
				navigationState.getAttitudeNB());

		//
		// VERIFY CLOSED-LOOP CONVERGENCE
		//

		EXPECT_LT(
			finalTruthError.magnitude(),
			0.02);

		EXPECT_LT(
			finalNavigationError.magnitude(),
			0.02);

		EXPECT_LT(
			finalTruth
			.getAngularVelocityB()
			.magnitude(),
			0.05);

		//
		// VERIFY NAVIGATION STILL TRACKS TRUTH
		//

		const math::Vector3 truthVsNavigationError =
			gnc::computeAttitudeErrorVectorB(
				finalTruth.getAttitudeNB(),
				navigationState.getAttitudeNB());

		EXPECT_LT(
			truthVsNavigationError.magnitude(),
			0.01);

		EXPECT_NEAR(
			navigationState
			.getPositionN()
			.x,
			finalTruth
			.getPositionN()
			.x,
			1e-9);

		EXPECT_NEAR(
			navigationState
			.getVelocityN()
			.x,
			finalTruth
			.getVelocityN()
			.x,
			1e-9);
	}
}