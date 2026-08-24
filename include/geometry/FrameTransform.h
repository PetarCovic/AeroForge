#pragma once

namespace aeroforge::math
{
    struct Vector3;
    struct Matrix;
    struct Quaternion;
}

namespace aeroforge::geometry
{
    math::Vector3 bodyToNavigation(
        const math::Vector3& vectorBody,
        const math::Quaternion& qNB);

    math::Vector3 navigationToBody(
        const math::Vector3& vectorNavigation,
        const math::Quaternion& qNB);

    math::Matrix bodyToNavigationMatrix(
        const math::Quaternion& qNB);

    math::Matrix navigationToBodyMatrix(
        const math::Quaternion& qNB);
}