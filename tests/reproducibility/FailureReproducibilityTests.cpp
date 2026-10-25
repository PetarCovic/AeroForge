#include <gtest/gtest.h>

#include <failures/ActuatorFaultInjector.h>
#include <failures/FaultSchedule.h>
#include <failures/GpsFaultInjector.h>
#include <failures/SensorFaultInjector.h>

#include <sensors/GpsModel.h>

#include <math/Vector3.h>

namespace aeroforge::reproducibility
{
	TEST(
		FailureReproducibilityTests,
		SensorStuckFaultReplaysExactly
	)
	{
		failures::SensorFaultInjector first(
			failures::FaultSchedule(1.0, 3.0),
			failures::SensorFaultType::StuckAt,
			math::Vector3()
		);

		failures::SensorFaultInjector second(
			failures::FaultSchedule(1.0, 3.0),
			failures::SensorFaultType::StuckAt,
			math::Vector3()
		);

		for (int i = 0; i < 8; ++i)
		{
			double timeS =
				i * 0.5;

			math::Vector3 measurement(
				static_cast<double>(i),
				2.0 * i,
				3.0 * i
			);

			auto a =
				first.apply(
					timeS,
					measurement
				);

			auto b =
				second.apply(
					timeS,
					measurement
				);

			EXPECT_EQ(
				a.has_value(),
				b.has_value()
			);

			if (a.has_value()
				&& b.has_value())
			{
				EXPECT_DOUBLE_EQ(
					a->x,
					b->x
				);

				EXPECT_DOUBLE_EQ(
					a->y,
					b->y
				);

				EXPECT_DOUBLE_EQ(
					a->z,
					b->z
				);
			}
		}
	}


	TEST(
		FailureReproducibilityTests,
		GpsDropoutReplaysExactly
	)
	{
		failures::GpsFaultInjector first(
			failures::FaultSchedule(1.0, 3.0),
			failures::GpsFaultType::Dropout,
			math::Vector3(),
			math::Vector3()
		);

		failures::GpsFaultInjector second(
			failures::FaultSchedule(1.0, 3.0),
			failures::GpsFaultType::Dropout,
			math::Vector3(),
			math::Vector3()
		);

		for (int i = 0; i < 8; ++i)
		{
			sensors::GpsMeasurement measurement{
				i * 0.5,
				math::Vector3(i, 0.0, 0.0),
				math::Vector3(1.0, 0.0, 0.0)
			};

			auto a = first.apply(measurement);
			auto b = second.apply(measurement);

			EXPECT_EQ(
				a.has_value(),
				b.has_value()
			);

			if (a.has_value()
				&& b.has_value())
			{
				EXPECT_DOUBLE_EQ(
					a->timeS,
					b->timeS
				);

				EXPECT_DOUBLE_EQ(
					a->positionN.x,
					b->positionN.x
				);
			}
		}
	}


	TEST(
		FailureReproducibilityTests,
		ActuatorStuckFaultReplaysExactly
	)
	{
		failures::ActuatorFaultInjector first(
			failures::FaultSchedule(1.0, 3.0),
			failures::ActuatorFaultType::StuckAt,
			math::Vector3()
		);

		failures::ActuatorFaultInjector second(
			failures::FaultSchedule(1.0, 3.0),
			failures::ActuatorFaultType::StuckAt,
			math::Vector3()
		);

		for (int i = 0; i < 8; ++i)
		{
			double timeS =
				i * 0.5;

			math::Vector3 input(
				i,
				-i,
				2.0 * i
			);

			math::Vector3 a =
				first.apply(timeS, input);

			math::Vector3 b =
				second.apply(timeS, input);

			EXPECT_DOUBLE_EQ(a.x, b.x);
			EXPECT_DOUBLE_EQ(a.y, b.y);
			EXPECT_DOUBLE_EQ(a.z, b.z);
		}
	}
}