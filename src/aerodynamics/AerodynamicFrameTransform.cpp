#include <aerodynamics/AerodynamicFrameTransform.h>
#include <math/Matrix.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::aerodynamics
{
    math::Vector3 transformWindToBody(
        const math::Vector3& vectorWind,
        double alphaRad,
        double betaRad)
    {
		if (!vectorWind.isFinite())
		{
			throw std::invalid_argument("Input vectorWind contains non-finite values.");
		}

		if (!std::isfinite(alphaRad) || !std::isfinite(betaRad))
		{
			throw std::invalid_argument("Input angles must be finite.");
		}

		math::Matrix transformationMatrix(3, 3,
			{
				std::cos(alphaRad) * std::cos(betaRad), -std::cos(alphaRad) * std::sin(betaRad), -std::sin(alphaRad),
				std::sin(betaRad), std::cos(betaRad), 0.0,
				std::sin(alphaRad) * std::cos(betaRad), -std::sin(alphaRad) * std::sin(betaRad), std::cos(alphaRad)
			});

		math::Vector3 bodyVector = transformationMatrix.multVector(vectorWind);

		if (!bodyVector.isFinite())
		{
			throw std::overflow_error("Resulting body vector contains non-finite values.");
		}

		return bodyVector;
    }
}