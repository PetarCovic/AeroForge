#include <gtest/gtest.h>

#include <math/Vector3.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aeroforge::math
{
    namespace
    {
        constexpr double Tolerance = 1e-9;

        void expectVectorNear(
            const Vector3& actual,
            const Vector3& expected,
            double tolerance = Tolerance)
        {
            EXPECT_NEAR(actual.x, expected.x, tolerance);
            EXPECT_NEAR(actual.y, expected.y, tolerance);
            EXPECT_NEAR(actual.z, expected.z, tolerance);
        }
    }

    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, DefaultConstructorCreatesZeroVector)
    {
        Vector3 vector;

        EXPECT_DOUBLE_EQ(vector.x, 0.0);
        EXPECT_DOUBLE_EQ(vector.y, 0.0);
        EXPECT_DOUBLE_EQ(vector.z, 0.0);
    }

    TEST(Vector3Tests, ParameterizedConstructorStoresComponents)
    {
        Vector3 vector(1.5, -2.0, 3.25);

        EXPECT_DOUBLE_EQ(vector.x, 1.5);
        EXPECT_DOUBLE_EQ(vector.y, -2.0);
        EXPECT_DOUBLE_EQ(vector.z, 3.25);
    }

    // -------------------------------------------------------------------------
    // Addition
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, AddReturnsVectorSum)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(4.0, 5.0, 6.0);

        Vector3 result = a.add(b);

        expectVectorNear(result, Vector3(5.0, 7.0, 9.0));
    }

    TEST(Vector3Tests, AdditionOperatorReturnsVectorSum)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(4.0, 5.0, 6.0);

        Vector3 result = a + b;

        expectVectorNear(result, Vector3(5.0, 7.0, 9.0));

        // + should not modify its operands.
        expectVectorNear(a, Vector3(1.0, 2.0, 3.0));
        expectVectorNear(b, Vector3(4.0, 5.0, 6.0));
    }

    TEST(Vector3Tests, AdditionAssignmentModifiesVector)
    {
        Vector3 vector(1.0, 2.0, 3.0);

        vector += Vector3(4.0, 5.0, 6.0);

        expectVectorNear(vector, Vector3(5.0, 7.0, 9.0));
    }

    // -------------------------------------------------------------------------
    // Subtraction
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, SubReturnsVectorDifference)
    {
        Vector3 a(5.0, 7.0, 9.0);
        Vector3 b(1.0, 2.0, 3.0);

        Vector3 result = a.sub(b);

        expectVectorNear(result, Vector3(4.0, 5.0, 6.0));
    }

    TEST(Vector3Tests, SubtractionOperatorReturnsVectorDifference)
    {
        Vector3 a(5.0, 7.0, 9.0);
        Vector3 b(1.0, 2.0, 3.0);

        Vector3 result = a - b;

        expectVectorNear(result, Vector3(4.0, 5.0, 6.0));

        expectVectorNear(a, Vector3(5.0, 7.0, 9.0));
        expectVectorNear(b, Vector3(1.0, 2.0, 3.0));
    }

    TEST(Vector3Tests, SubtractionAssignmentModifiesVector)
    {
        Vector3 vector(5.0, 7.0, 9.0);

        vector -= Vector3(1.0, 2.0, 3.0);

        expectVectorNear(vector, Vector3(4.0, 5.0, 6.0));
    }

    // -------------------------------------------------------------------------
    // Scalar multiplication
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, ScaleMultipliesAllComponents)
    {
        Vector3 vector(1.0, -2.0, 3.0);

        Vector3 result = vector.scale(2.5);

        expectVectorNear(result, Vector3(2.5, -5.0, 7.5));
    }

    TEST(Vector3Tests, VectorTimesScalarWorks)
    {
        Vector3 vector(1.0, -2.0, 3.0);

        Vector3 result = vector * 2.0;

        expectVectorNear(result, Vector3(2.0, -4.0, 6.0));
    }

    TEST(Vector3Tests, ScalarTimesVectorWorks)
    {
        Vector3 vector(1.0, -2.0, 3.0);

        Vector3 result = 2.0 * vector;

        expectVectorNear(result, Vector3(2.0, -4.0, 6.0));
    }

    TEST(Vector3Tests, MultiplicationAssignmentModifiesVector)
    {
        Vector3 vector(1.0, -2.0, 3.0);

        vector *= 3.0;

        expectVectorNear(vector, Vector3(3.0, -6.0, 9.0));
    }

    // -------------------------------------------------------------------------
    // Scalar division
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, DivDividesAllComponents)
    {
        Vector3 vector(2.0, 4.0, 6.0);

        Vector3 result = vector.div(2.0);

        expectVectorNear(result, Vector3(1.0, 2.0, 3.0));
    }

    TEST(Vector3Tests, DivisionOperatorDividesAllComponents)
    {
        Vector3 vector(2.0, 4.0, 6.0);

        Vector3 result = vector / 2.0;

        expectVectorNear(result, Vector3(1.0, 2.0, 3.0));
    }

    TEST(Vector3Tests, DivisionAssignmentModifiesVector)
    {
        Vector3 vector(2.0, 4.0, 6.0);

        vector /= 2.0;

        expectVectorNear(vector, Vector3(1.0, 2.0, 3.0));
    }

    TEST(Vector3Tests, DivByZeroThrows)
    {
        Vector3 vector(1.0, 2.0, 3.0);

        EXPECT_THROW(vector.div(0.0), std::invalid_argument);
    }

    TEST(Vector3Tests, DivisionOperatorByZeroThrows)
    {
        Vector3 vector(1.0, 2.0, 3.0);

        EXPECT_THROW(vector / 0.0, std::invalid_argument);
    }

    TEST(Vector3Tests, DivisionAssignmentByZeroThrows)
    {
        Vector3 vector(1.0, 2.0, 3.0);

        EXPECT_THROW(vector /= 0.0, std::invalid_argument);
    }

    // -------------------------------------------------------------------------
    // Negation
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, NegationReturnsOppositeVector)
    {
        Vector3 vector(1.0, -2.0, 3.0);

        Vector3 result = vector.negation();

        expectVectorNear(result, Vector3(-1.0, 2.0, -3.0));
    }

    TEST(Vector3Tests, UnaryMinusReturnsOppositeVector)
    {
        Vector3 vector(1.0, -2.0, 3.0);

        Vector3 result = -vector;

        expectVectorNear(result, Vector3(-1.0, 2.0, -3.0));
    }

    // -------------------------------------------------------------------------
    // Dot product
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, DotProductCalculatesCorrectResult)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(4.0, 5.0, 6.0);

        EXPECT_DOUBLE_EQ(a.dot(b), 32.0);
    }

    TEST(Vector3Tests, DotProductOfOrthogonalVectorsIsZero)
    {
        EXPECT_DOUBLE_EQ(
            Vector3::unitX().dot(Vector3::unitY()),
            0.0);
    }

    TEST(Vector3Tests, DotProductWithItselfEqualsSquaredMagnitude)
    {
        Vector3 vector(3.0, 4.0, 5.0);

        EXPECT_DOUBLE_EQ(
            vector.dot(vector),
            vector.squaredMagnitude());
    }

    // -------------------------------------------------------------------------
    // Cross product
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, UnitXCrossUnitYEqualsUnitZ)
    {
        Vector3 result =
            Vector3::unitX().cross(Vector3::unitY());

        expectVectorNear(result, Vector3::unitZ());
    }

    TEST(Vector3Tests, UnitYCrossUnitXEqualsNegativeUnitZ)
    {
        Vector3 result =
            Vector3::unitY().cross(Vector3::unitX());

        expectVectorNear(result, -Vector3::unitZ());
    }

    TEST(Vector3Tests, VectorCrossItselfEqualsZero)
    {
        Vector3 vector(3.0, -2.0, 5.0);

        Vector3 result = vector.cross(vector);

        expectVectorNear(result, Vector3(0.0, 0.0, 0.0));
    }

    TEST(Vector3Tests, CrossProductIsPerpendicularToBothInputs)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(4.0, -5.0, 6.0);

        Vector3 cross = a.cross(b);

        EXPECT_NEAR(cross.dot(a), 0.0, Tolerance);
        EXPECT_NEAR(cross.dot(b), 0.0, Tolerance);
    }

    // -------------------------------------------------------------------------
    // Magnitude
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, MagnitudeCalculatesCorrectLength)
    {
        Vector3 vector(3.0, 4.0, 0.0);

        EXPECT_DOUBLE_EQ(vector.magnitude(), 5.0);
    }

    TEST(Vector3Tests, SquaredMagnitudeCalculatesCorrectValue)
    {
        Vector3 vector(3.0, 4.0, 0.0);

        EXPECT_DOUBLE_EQ(vector.squaredMagnitude(), 25.0);
    }

    TEST(Vector3Tests, ZeroVectorHasZeroMagnitude)
    {
        Vector3 vector;

        EXPECT_DOUBLE_EQ(vector.magnitude(), 0.0);
        EXPECT_DOUBLE_EQ(vector.squaredMagnitude(), 0.0);
    }

    // -------------------------------------------------------------------------
    // Normalization
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, NormalizeProducesUnitVector)
    {
        Vector3 vector(3.0, 4.0, 0.0);

        Vector3 normalized = vector.normalize();

        expectVectorNear(
            normalized,
            Vector3(0.6, 0.8, 0.0));

        EXPECT_NEAR(normalized.magnitude(), 1.0, Tolerance);
    }

    TEST(Vector3Tests, NormalizeDoesNotModifyOriginalVector)
    {
        Vector3 vector(3.0, 4.0, 0.0);

        Vector3 normalized = vector.normalize();

        (void)normalized;

        expectVectorNear(vector, Vector3(3.0, 4.0, 0.0));
    }

    TEST(Vector3Tests, NormalizeZeroVectorThrows)
    {
        Vector3 vector;

        EXPECT_THROW(vector.normalize(), std::domain_error);
    }

    TEST(Vector3Tests, IsNormalizedReturnsTrueForUnitVector)
    {
        Vector3 vector(3.0 / 5.0, 4.0 / 5.0, 0.0);

        EXPECT_TRUE(vector.isNormalized());
    }

    TEST(Vector3Tests, IsNormalizedReturnsFalseForNonUnitVector)
    {
        Vector3 vector(3.0, 4.0, 0.0);

        EXPECT_FALSE(vector.isNormalized());
    }

    // -------------------------------------------------------------------------
    // Projection and rejection
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, ProjectOntoXAxisProducesXAxisComponent)
    {
        Vector3 vector(3.0, 4.0, 5.0);

        Vector3 result = vector.project(Vector3::unitX());

        expectVectorNear(result, Vector3(3.0, 0.0, 0.0));
    }

    TEST(Vector3Tests, ProjectOntoZeroVectorThrows)
    {
        Vector3 vector(1.0, 2.0, 3.0);
        Vector3 zero;

        EXPECT_THROW(vector.project(zero), std::domain_error);
    }

    TEST(Vector3Tests, RejectRemovesProjectedComponent)
    {
        Vector3 vector(3.0, 4.0, 5.0);

        Vector3 result = vector.reject(Vector3::unitX());

        expectVectorNear(result, Vector3(0.0, 4.0, 5.0));
    }

    TEST(Vector3Tests, ProjectionPlusRejectionReconstructsOriginalVector)
    {
        Vector3 vector(3.0, 4.0, 5.0);
        Vector3 axis(1.0, 2.0, 0.0);

        Vector3 projection = vector.project(axis);
        Vector3 rejection = vector.reject(axis);

        expectVectorNear(projection + rejection, vector);
    }

    // -------------------------------------------------------------------------
    // Finite checks
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, IsFiniteReturnsTrueForFiniteVector)
    {
        Vector3 vector(1.0, -2.0, 3.0);

        EXPECT_TRUE(vector.isFinite());
    }

    TEST(Vector3Tests, IsFiniteReturnsFalseForInfinity)
    {
        Vector3 vector(
            1.0,
            std::numeric_limits<double>::infinity(),
            3.0);

        EXPECT_FALSE(vector.isFinite());
    }

    TEST(Vector3Tests, IsFiniteReturnsFalseForNaN)
    {
        Vector3 vector(
            1.0,
            std::numeric_limits<double>::quiet_NaN(),
            3.0);

        EXPECT_FALSE(vector.isFinite());
    }

    // -------------------------------------------------------------------------
    // Approximate equality
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, IsApproxReturnsTrueWithinTolerance)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(1.000001, 2.000001, 3.000001);

        EXPECT_TRUE(a.isApprox(b, 1e-5));
    }

    TEST(Vector3Tests, IsApproxReturnsFalseOutsideTolerance)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(1.1, 2.0, 3.0);

        EXPECT_FALSE(a.isApprox(b, 1e-5));
    }

    TEST(Vector3Tests, IsApproxRejectsZeroTolerance)
    {
        Vector3 a;
        Vector3 b;

        EXPECT_THROW(
            a.isApprox(b, 0.0),
            std::invalid_argument);
    }

    TEST(Vector3Tests, IsApproxRejectsNegativeTolerance)
    {
        Vector3 a;
        Vector3 b;

        EXPECT_THROW(
            a.isApprox(b, -1e-6),
            std::invalid_argument);
    }

    // -------------------------------------------------------------------------
    // Unit basis vectors
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, UnitXReturnsXAxisBasisVector)
    {
        expectVectorNear(
            Vector3::unitX(),
            Vector3(1.0, 0.0, 0.0));
    }

    TEST(Vector3Tests, UnitYReturnsYAxisBasisVector)
    {
        expectVectorNear(
            Vector3::unitY(),
            Vector3(0.0, 1.0, 0.0));
    }

    TEST(Vector3Tests, UnitZReturnsZAxisBasisVector)
    {
        expectVectorNear(
            Vector3::unitZ(),
            Vector3(0.0, 0.0, 1.0));
    }

    // -------------------------------------------------------------------------
    // Reflection
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, ReflectAcrossYNormalFlipsYComponent)
    {
        Vector3 vector(1.0, -1.0, 0.0);
        Vector3 normal = Vector3::unitY();

        Vector3 result = vector.reflect(normal);

        expectVectorNear(result, Vector3(1.0, 1.0, 0.0));
    }

    TEST(Vector3Tests, ReflectRejectsNonNormalizedNormal)
    {
        Vector3 vector(1.0, -1.0, 0.0);
        Vector3 normal(0.0, 2.0, 0.0);

        EXPECT_THROW(
            vector.reflect(normal),
            std::domain_error);
    }

    // -------------------------------------------------------------------------
    // Linear interpolation
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, LerpAtZeroReturnsStartingVector)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(5.0, 6.0, 7.0);

        expectVectorNear(a.lerp(b, 0.0), a);
    }

    TEST(Vector3Tests, LerpAtOneReturnsEndingVector)
    {
        Vector3 a(1.0, 2.0, 3.0);
        Vector3 b(5.0, 6.0, 7.0);

        expectVectorNear(a.lerp(b, 1.0), b);
    }

    TEST(Vector3Tests, LerpAtHalfReturnsMidpoint)
    {
        Vector3 a(0.0, 0.0, 0.0);
        Vector3 b(10.0, 20.0, 30.0);

        Vector3 result = a.lerp(b, 0.5);

        expectVectorNear(result, Vector3(5.0, 10.0, 15.0));
    }

    // -------------------------------------------------------------------------
    // Indexing
    // -------------------------------------------------------------------------

    TEST(Vector3Tests, ConstIndexOperatorReturnsCorrectComponents)
    {
        const Vector3 vector(1.0, 2.0, 3.0);

        EXPECT_DOUBLE_EQ(vector[0], 1.0);
        EXPECT_DOUBLE_EQ(vector[1], 2.0);
        EXPECT_DOUBLE_EQ(vector[2], 3.0);
    }

    TEST(Vector3Tests, MutableIndexOperatorCanModifyComponents)
    {
        Vector3 vector;

        vector[0] = 1.0;
        vector[1] = 2.0;
        vector[2] = 3.0;

        expectVectorNear(vector, Vector3(1.0, 2.0, 3.0));
    }

    TEST(Vector3Tests, IndexOperatorThrowsForIndexAboveTwo)
    {
        Vector3 vector;

        EXPECT_THROW(vector[3], std::out_of_range);
    }

    TEST(Vector3Tests, ConstIndexOperatorThrowsForIndexAboveTwo)
    {
        const Vector3 vector;

        EXPECT_THROW(vector[3], std::out_of_range);
    }
}