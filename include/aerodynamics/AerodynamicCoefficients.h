#pragma once

namespace aeroforge::aerodynamics
{
	class AerodynamicCoefficients
	{
	public:
		AerodynamicCoefficients(
			double dragCoefficient,
			double liftCoefficient,
			double sideForceCoefficient);

		double getDragCoefficient() const;

		double getLiftCoefficient() const;

		double getSideForceCoefficient() const;

	private:
		double dragCoefficient;
		double liftCoefficient;
		double sideForceCoefficient;
	};
}