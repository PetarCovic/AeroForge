#pragma once

namespace aeroforge::aerodynamics
{
	class AerodynamicReference
	{
	public:
		AerodynamicReference(
			double referenceAreaM2,
			double referenceLengthM);

		double getReferenceAreaM2() const;

		double getReferenceLengthM() const;

	private:
		double referenceAreaM2;
		double referenceLengthM;
	};
}