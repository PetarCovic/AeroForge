#include <visualization/VisualizationInterpolator.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::visualization
{
	namespace
	{
		math::Vector3 interpolateVector(
			const math::Vector3& first,
			const math::Vector3& second,
			double alpha)
		{
			return
				first +
				(second - first) *
				alpha;
		}
	}

	VisualizationSnapshot VisualizationInterpolator::interpolate(
		const VisualizationSnapshot& first,
		const VisualizationSnapshot& second,
		double simulationTimeS)
	{
		if (
			!std::isfinite(first.simulationTimeS) ||
			!std::isfinite(second.simulationTimeS))
		{
			throw std::invalid_argument(
				"Snapshot times must be finite");
		}

		if (!std::isfinite(simulationTimeS))
		{
			throw std::invalid_argument(
				"simulationTimeS must be finite");
		}

		if (
			second.simulationTimeS <
			first.simulationTimeS)
		{
			throw std::invalid_argument(
				"Second snapshot must not precede first snapshot");
		}

		if (
			second.simulationTimeS ==
			first.simulationTimeS)
		{
			return first;
		}

		if (
			simulationTimeS <=
			first.simulationTimeS)
		{
			return first;
		}

		if (
			simulationTimeS >=
			second.simulationTimeS)
		{
			return second;
		}

		const double alpha =
			(simulationTimeS -
				first.simulationTimeS) /
			(second.simulationTimeS -
				first.simulationTimeS);

		return VisualizationSnapshot{
			simulationTimeS,

			interpolateVector(
				first.truthPositionN,
				second.truthPositionN,
				alpha),

			interpolateVector(
				first.truthVelocityN,
				second.truthVelocityN,
				alpha),

			first.truthAttitudeNB.slerp(
				second.truthAttitudeNB,
				alpha),

			interpolateVector(
				first.truthAngularVelocityB,
				second.truthAngularVelocityB,
				alpha),

			interpolateVector(
				first.estimatedPositionN,
				second.estimatedPositionN,
				alpha),

			interpolateVector(
				first.estimatedVelocityN,
				second.estimatedVelocityN,
				alpha),

			first.estimatedAttitudeNB.slerp(
				second.estimatedAttitudeNB,
				alpha),

			interpolateVector(
				first.estimatedAngularVelocityB,
				second.estimatedAngularVelocityB,
				alpha),

			interpolateVector(
				first.attitudeErrorB,
				second.attitudeErrorB,
				alpha),

			interpolateVector(
				first.commandedMomentB,
				second.commandedMomentB,
				alpha),

			interpolateVector(
				first.actualMomentB,
				second.actualMomentB,
				alpha)
		};
	}
}