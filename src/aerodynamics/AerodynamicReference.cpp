#include <aerodynamics/AerodynamicReference.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::aerodynamics
{
	AerodynamicReference::AerodynamicReference(
		double referenceAreaM2,
		double referenceLengthM)
		: referenceAreaM2(referenceAreaM2),
		  referenceLengthM(referenceLengthM)
	{
		if (referenceLengthM <= 0.0 || !std::isfinite(referenceLengthM))
		{
			throw std::invalid_argument("Reference length must be a finite positive value.");
		}

		if (referenceAreaM2 <= 0.0 || !std::isfinite(referenceAreaM2))
		{
			throw std::invalid_argument("Reference area must be a finite positive value.");
		}
	}

	double AerodynamicReference::getReferenceAreaM2() const
	{
		return referenceAreaM2;
	}

	double AerodynamicReference::getReferenceLengthM() const
	{
		return referenceLengthM;
	}
}