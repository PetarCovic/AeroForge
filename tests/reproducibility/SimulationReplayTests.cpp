#include <gtest/gtest.h>

#include <reproducibility/ReplayConfig.h>
#include <reproducibility/SeedManager.h>

#include <actuators/BodyMomentActuator.h>

#include <dynamics/RotationalKinematics.h>

#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeSetpoint.h>
#include <gnc/HoldAttitudeGuidance.h>

#include <navigation/InertialNavigator.h>
#include <navigation/NavigationState.h>

#include <sensors/AccelerometerModel.h>
#include <sensors/GyroscopeModel.h>
#include <sensors/ImuModel.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>

namespace aeroforge::reproducibility
{
	namespace
	{
		struct ReplayResult
		{
			math::Vector3 positionN;
			math::Vector3 velocityN;
			math::Quaternion attitudeNB;
			math::Vector3 angularVelocityB;
		};


		ReplayResult runReplay(
			const ReplayConfig& config)
		{
			SeedManager seedManager(
				config.getTrialSeed()
			);

			std::uint32_t accelSeed =
				seedManager.deriveSeed(
					config.getTrialIndex(),
					1
				);

			std::uint32_t gyroSeed =
				seedManager.deriveSeed(
					config.getTrialIndex(),
					2
				);

			sensors::AccelerometerModel accelerometer(
				math::Vector3(),
				math::Vector3(
					0.001,
					0.001,
					0.001
				),
				math::Vector3(
					1000.0,
					1000.0,
					1000.0
				),
				accelSeed
			);

			sensors::GyroscopeModel gyroscope(
				math::Vector3(),
				math::Vector3(
					0.0001,
					0.0001,
					0.0001
				),
				math::Vector3(
					100.0,
					100.0,
					100.0
				),
				gyroSeed
			);

			sensors::ImuModel imu(
				accelerometer,
				gyroscope
			);

			navigation::InertialNavigator navigator;

			navigation::NavigationState navigationState(
				config.getInitialPositionN(),
				config.getInitialVelocityN(),
				config.getInitialAttitudeNB(),
				config.getInitialAngularVelocityB()
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
				math::Vector3(
					8.0,
					8.0,
					8.0
				),
				math::Vector3(
					5.0,
					5.0,
					5.0
				),
				math::Vector3(
					20.0,
					20.0,
					20.0
				)
			);

			actuators::BodyMomentActuator actuator(
				math::Vector3(
					20.0,
					20.0,
					20.0
				),
				math::Vector3(
					1000.0,
					1000.0,
					1000.0
				)
			);

			const double dtS =
				config.getDtS();

			const int steps =
				500;

			const double inertiaZ =
				2.0;

			for (int i = 0;
				i < steps;
				++i)
			{
				double timeS =
					i * dtS;

				sensors::ImuMeasurement measurement =
					imu.sample(
						timeS,
						math::Vector3(
							0.0,
							0.0,
							0.0
						),
						navigationState
						.getAngularVelocityB()
					);

				navigationState =
					navigator.propagate(
						navigationState,
						measurement,
						math::Vector3(),
						dtS
					);

				auto setpoint =
					guidance.getSetpoint(
						timeS
					);

				math::Vector3 command =
					controller.computeMomentCommandB(
						setpoint,
						navigationState
					);

				math::Vector3 moment =
					actuator.update(
						command,
						dtS
					);

				math::Vector3 omega =
					navigationState
					.getAngularVelocityB();

				math::Vector3 nextOmega(
					omega.x,
					omega.y,
					omega.z
					+ moment.z
					/ inertiaZ
					* dtS
				);

				math::Quaternion q =
					navigationState
					.getAttitudeNB();

				math::Quaternion qDot =
					dynamics::
					computeQuaternionDerivativeNB(
						q,
						nextOmega
					);

				math::Quaternion nextQ(
					q.w + qDot.w * dtS,
					q.x + qDot.x * dtS,
					q.y + qDot.y * dtS,
					q.z + qDot.z * dtS
				);

				nextQ =
					nextQ.normalized();

				navigationState =
					navigation::NavigationState(
						navigationState
						.getPositionN(),
						navigationState
						.getVelocityN(),
						nextQ,
						nextOmega
					);
			}

			return ReplayResult{
				navigationState.getPositionN(),
				navigationState.getVelocityN(),
				navigationState.getAttitudeNB(),
				navigationState.getAngularVelocityB()
			};
		}
	}


	TEST(
		SimulationReplayTests,
		IdenticalReplayConfigProducesIdenticalFinalState
	)
	{
		const double yaw =
			20.0
			* 3.14159265358979323846
			/ 180.0;

		ReplayConfig config(
			42,
			123456u,
			0.005,
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				std::cos(yaw / 2.0),
				0.0,
				0.0,
				std::sin(yaw / 2.0)
			),
			math::Vector3(),
			{
				{"massScale", 1.02},
				{"thrustScale", 0.98}
			}
		);

		ReplayResult first =
			runReplay(config);

		ReplayResult second =
			runReplay(config);


		EXPECT_DOUBLE_EQ(
			first.positionN.x,
			second.positionN.x
		);

		EXPECT_DOUBLE_EQ(
			first.positionN.y,
			second.positionN.y
		);

		EXPECT_DOUBLE_EQ(
			first.positionN.z,
			second.positionN.z
		);


		EXPECT_DOUBLE_EQ(
			first.velocityN.x,
			second.velocityN.x
		);

		EXPECT_DOUBLE_EQ(
			first.velocityN.y,
			second.velocityN.y
		);

		EXPECT_DOUBLE_EQ(
			first.velocityN.z,
			second.velocityN.z
		);


		EXPECT_DOUBLE_EQ(
			first.attitudeNB.w,
			second.attitudeNB.w
		);

		EXPECT_DOUBLE_EQ(
			first.attitudeNB.x,
			second.attitudeNB.x
		);

		EXPECT_DOUBLE_EQ(
			first.attitudeNB.y,
			second.attitudeNB.y
		);

		EXPECT_DOUBLE_EQ(
			first.attitudeNB.z,
			second.attitudeNB.z
		);


		EXPECT_DOUBLE_EQ(
			first.angularVelocityB.x,
			second.angularVelocityB.x
		);

		EXPECT_DOUBLE_EQ(
			first.angularVelocityB.y,
			second.angularVelocityB.y
		);

		EXPECT_DOUBLE_EQ(
			first.angularVelocityB.z,
			second.angularVelocityB.z
		);
	}


	TEST(
		SimulationReplayTests,
		DifferentTrialSeedChangesNoisyReplay
	)
	{
		ReplayConfig firstConfig(
			42,
			111u,
			0.005,
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0
			),
			math::Vector3(),
			{}
		);

		ReplayConfig secondConfig(
			42,
			222u,
			0.005,
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0
			),
			math::Vector3(),
			{}
		);

		ReplayResult first =
			runReplay(firstConfig);

		ReplayResult second =
			runReplay(secondConfig);

		bool differenceFound =
			first.positionN.x
			!= second.positionN.x
			|| first.positionN.y
			!= second.positionN.y
			|| first.positionN.z
			!= second.positionN.z
			|| first.attitudeNB.w
			!= second.attitudeNB.w
			|| first.attitudeNB.x
			!= second.attitudeNB.x
			|| first.attitudeNB.y
			!= second.attitudeNB.y
			|| first.attitudeNB.z
			!= second.attitudeNB.z;

		EXPECT_TRUE(differenceFound);
	}
}