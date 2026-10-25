#pragma once

namespace aeroforge::rocket
{
	class PropellantTank
	{
	public:
		explicit PropellantTank(double capacityKg, double initialPropellantMassKg);

		double getCapacityKg() const;

		double getPropellantMassKg() const;

		double getPropellantFraction() const;

		bool isEmpty() const;

		double consume(double massKg);

	private:
		double capacityKg;
		double propellantMassKg;
	};
}