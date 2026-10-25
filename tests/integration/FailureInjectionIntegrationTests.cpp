#include <gtest/gtest.h>

#include <failures/FaultSchedule.h>
#include <failures/SensorFaultInjector.h>
#include <failures/GpsFaultInjector.h>
#include <failures/ActuatorFaultInjector.h>

#include <navigation/NavigationState.h>
#include <navigation/InertialNavigator.h>
#include <navigation/GpsCorrector.h>

#include <sensors/ImuModel.h>
#include <sensors/GpsModel.h>

#include <actuators/BodyMomentActuator.h>

#include <gnc/AttitudeSetpoint.h>
#include <gnc/HoldAttitudeGuidance.h>
#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeError.h>

#include <dynamics/RotationalKinematics.h>

#include <math/Vector3.h>
#include <math/Quaternion.h>

#include <cmath>

namespace aeroforge::failures
{
	namespace
	{
		constexpr double PI =
			3.14159265358979323846;

		constexpr double TOLERANCE =
			1e-9;
	}


	TEST(
		FailureInjectionIntegrationTests,
		AccelerometerBiasProducesNavigationDriftOnlyDuringFaultWindow
	)
	{
		SensorFaultInjector accelerometerFault(
			FaultSchedule(0.5, 1.0),
			SensorFaultType::Bias,
			math::Vector3(1.0, 0.0, 0.0)
		);

		navigation::InertialNavigator navigator;

		navigation::NavigationState state(
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0),
			math::Quaternion(1.0, 0.0, 0.0, 0.0),
			math::Vector3(0.0, 0.0, 0.0)
		);

		const math::Vector3 gravityN(
			0.0,
			0.0,
			0.0
		);

		const double dtS = 0.25;

		for (int i = 0; i < 6; ++i)
		{
			double timeS = i * dtS;

			auto faultedSpecificForce =
				accelerometerFault.apply(
					timeS,
					math::Vector3(0.0, 0.0, 0.0)
				);

			ASSERT_TRUE(faultedSpecificForce.has_value());

			sensors::ImuMeasurement measurement{
				timeS,
				*faultedSpecificForce,
				math::Vector3(0.0, 0.0, 0.0)
			};

			state = navigator.propagate(
				state,
				measurement,
				gravityN,
				dtS
			);
		}

		// Fault is active for exactly 0.5 s:
		//
		// acceleration = 1 m/s^2
		// final velocity = 0.5 m/s
		//
		// 0.125 m traveled while accelerating +
		// 0.250 m while coasting afterward.
		EXPECT_NEAR(
			state.getVelocityN().x,
			0.5,
			TOLERANCE
		);

		EXPECT_NEAR(
			state.getPositionN().x,
			0.375,
			TOLERANCE
		);

		EXPECT_NEAR(
			state.getVelocityN().y,
			0.0,
			TOLERANCE
		);

		EXPECT_NEAR(
			state.getVelocityN().z,
			0.0,
			TOLERANCE
		);
	}


	TEST(
		FailureInjectionIntegrationTests,
		GpsDropoutPreventsCorrectionAndCorrectionResumesAfterRecovery
	)
	{
		GpsFaultInjector gpsFault(
			FaultSchedule(1.0, 2.0),
			GpsFaultType::Dropout,
			math::Vector3(),
			math::Vector3()
		);

		navigation::GpsCorrector corrector(
			1.0,
			1.0
		);

		navigation::NavigationState state(
			math::Vector3(100.0, 0.0, 0.0),
			math::Vector3(10.0, 0.0, 0.0),
			math::Quaternion(1.0, 0.0, 0.0, 0.0),
			math::Vector3()
		);

		auto droppedMeasurement =
			gpsFault.apply(
				sensors::GpsMeasurement{
					1.5,
					math::Vector3(0.0, 0.0, 0.0),
					math::Vector3(0.0, 0.0, 0.0)
				}
			);

		EXPECT_FALSE(droppedMeasurement.has_value());

		// Without a measurement, navigation cannot perform
		// the GPS correction.
		EXPECT_DOUBLE_EQ(
			state.getPositionN().x,
			100.0
		);

		EXPECT_DOUBLE_EQ(
			state.getVelocityN().x,
			10.0
		);

		auto recoveredMeasurement =
			gpsFault.apply(
				sensors::GpsMeasurement{
					2.0,
					math::Vector3(0.0, 0.0, 0.0),
					math::Vector3(0.0, 0.0, 0.0)
				}
			);

		ASSERT_TRUE(recoveredMeasurement.has_value());

		state = corrector.correct(
			state,
			*recoveredMeasurement
		);

		EXPECT_NEAR(
			state.getPositionN().x,
			0.0,
			TOLERANCE
		);

		EXPECT_NEAR(
			state.getVelocityN().x,
			0.0,
			TOLERANCE
		);
	}


	TEST(
		FailureInjectionIntegrationTests,
		LossOfActuatorEffectivenessReducesControllerAuthority
	)
	{
		double yawRad =
			10.0 * PI / 180.0;

		math::Quaternion currentAttitude(
			std::cos(yawRad / 2.0),
			0.0,
			0.0,
			std::sin(yawRad / 2.0)
		);

		navigation::NavigationState navigationState(
			math::Vector3(),
			math::Vector3(),
			currentAttitude,
			math::Vector3()
		);

		gnc::AttitudeSetpoint setpoint(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0
			),
			math::Vector3()
		);

		gnc::AttitudePdController controller(
			math::Vector3(8.0, 8.0, 8.0),
			math::Vector3(5.0, 5.0, 5.0),
			math::Vector3(20.0, 20.0, 20.0)
		);

		actuators::BodyMomentActuator actuator(
			math::Vector3(20.0, 20.0, 20.0),
			math::Vector3(1000.0, 1000.0, 1000.0)
		);

		math::Vector3 command =
			controller.computeMomentCommandB(
				setpoint,
				navigationState
			);

		math::Vector3 nominalMoment =
			actuator.update(
				command,
				0.1
			);

		ActuatorFaultInjector fault(
			FaultSchedule(1.0, 2.0),
			ActuatorFaultType::LossOfEffectiveness,
			math::Vector3(
				1.0,
				1.0,
				0.25
			)
		);

		math::Vector3 faultedMoment =
			fault.apply(
				1.5,
				nominalMoment
			);

		EXPECT_LT(
			nominalMoment.z,
			0.0
		);

		EXPECT_NEAR(
			faultedMoment.z,
			0.25 * nominalMoment.z,
			TOLERANCE
		);

		EXPECT_NEAR(
			faultedMoment.x,
			nominalMoment.x,
			TOLERANCE
		);

		EXPECT_NEAR(
			faultedMoment.y,
			nominalMoment.y,
			TOLERANCE
		);
	}


	TEST(
		FailureInjectionIntegrationTests,
		CompleteYawActuatorFailurePreventsAttitudeCorrection
	)
	{
		const double initialYawRad =
			20.0 * PI / 180.0;

		const double dtS =
			0.005;

		const int steps =
			100;

		auto runScenario =
			[&](double yawEffectiveness)
			{
				math::Quaternion initialAttitude(
					std::cos(initialYawRad / 2.0),
					0.0,
					0.0,
					std::sin(initialYawRad / 2.0)
				);

				navigation::NavigationState state(
					math::Vector3(),
					math::Vector3(),
					initialAttitude,
					math::Vector3()
				);

				gnc::AttitudeSetpoint target(
					math::Quaternion(
						1.0,
						0.0,
						0.0,
						0.0
					),
					math::Vector3()
				);

				gnc::HoldAttitudeGuidance guidance(
					target
				);

				gnc::AttitudePdController controller(
					math::Vector3(8.0, 8.0, 8.0),
					math::Vector3(5.0, 5.0, 5.0),
					math::Vector3(20.0, 20.0, 20.0)
				);

				actuators::BodyMomentActuator actuator(
					math::Vector3(20.0, 20.0, 20.0),
					math::Vector3(
						1000.0,
						1000.0,
						1000.0
					)
				);

				ActuatorFaultInjector fault(
					FaultSchedule(0.0, 10.0),
					ActuatorFaultType::
					LossOfEffectiveness,
					math::Vector3(
						1.0,
						1.0,
						yawEffectiveness
					)
				);

				const double inertiaZ =
					2.0;

				for (int i = 0; i < steps; ++i)
				{
					double timeS =
						i * dtS;

					gnc::AttitudeSetpoint setpoint =
						guidance.getSetpoint(
							timeS
						);

					math::Vector3 command =
						controller.computeMomentCommandB(
							setpoint,
							state
						);

					math::Vector3 actuatorMoment =
						actuator.update(
							command,
							dtS
						);

					math::Vector3 appliedMoment =
						fault.apply(
							timeS,
							actuatorMoment
						);

					math::Vector3 omega =
						state.getAngularVelocityB();

					math::Vector3 nextOmega(
						omega.x,
						omega.y,
						omega.z
						+ (appliedMoment.z
							/ inertiaZ)
						* dtS
					);

					const math::Quaternion&
						currentAttitude =
						state.getAttitudeNB();

					math::Quaternion qDot =
						dynamics::
						computeQuaternionDerivativeNB(
							currentAttitude,
							nextOmega
						);

					math::Quaternion nextAttitude(
						currentAttitude.w
						+ qDot.w * dtS,

						currentAttitude.x
						+ qDot.x * dtS,

						currentAttitude.y
						+ qDot.y * dtS,

						currentAttitude.z
						+ qDot.z * dtS
					);

					nextAttitude =
						nextAttitude.normalized();

					state =
						navigation::NavigationState(
							math::Vector3(),
							math::Vector3(),
							nextAttitude,
							nextOmega
						);
				}

				math::Vector3 finalError =
					gnc::computeAttitudeErrorVectorB(
						target.getAttitudeNB(),
						state.getAttitudeNB()
					);

				return std::abs(finalError.z);
			};


		double nominalFinalError =
			runScenario(1.0);

		double failedFinalError =
			runScenario(0.0);

		double initialError =
			2.0
			* std::sin(initialYawRad / 2.0);

		// Healthy controller must make progress.
		EXPECT_LT(
			nominalFinalError,
			initialError
		);

		// Complete loss of yaw authority should leave the
		// initial attitude essentially unchanged.
		EXPECT_NEAR(
			failedFinalError,
			initialError,
			1e-6
		);

		// The failed vehicle must perform worse than the
		// healthy vehicle.
		EXPECT_GT(
			failedFinalError,
			nominalFinalError
		);
	}
}