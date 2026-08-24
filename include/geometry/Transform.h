#pragma once

#include <math/Matrix.h>
#include <math/Vector3.h>

namespace aeroforge::geometry
{
	aeroforge::math::Matrix translation(double x, double y, double z);

	aeroforge::math::Matrix rotationX(double angleRadians);

	aeroforge::math::Matrix rotationY(double angleRadians);

	aeroforge::math::Matrix rotationZ(double angleRadians);

	aeroforge::math::Matrix rotationAxis(const aeroforge::math::Vector3& axis, double angleRadians);
}