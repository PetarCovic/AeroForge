#pragma once

namespace aeroforge::dynamics
{
	class MassProperties
	{
	public:
		explicit MassProperties(double massKg);

		double getMassKg() const;

		void setMassKg(double massKg);

	private:
		double massKg;
	};
}