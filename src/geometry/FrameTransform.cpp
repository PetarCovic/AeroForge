#include <geometry/FrameTransform.h>
#include <math/Vector3.h>
#include <math/Matrix.h>
#include <math/Quaternion.h>

namespace aeroforge::geometry
{
    math::Vector3 bodyToNavigation(const math::Vector3& vectorBody, const math::Quaternion& qNB)
    {
		return qNB.rotate(vectorBody);
    }

    math::Vector3 navigationToBody(const math::Vector3& vectorNavigation, const math::Quaternion& qNB)
    {
		return qNB.rotateInverse(vectorNavigation);
    }

    math::Matrix bodyToNavigationMatrix(const math::Quaternion& qNB)
    {
		return qNB.toRotationMatrix();
    }

    math::Matrix navigationToBodyMatrix(const math::Quaternion& qNB)
    {
		return qNB.toRotationMatrix().transpose();
    }
}