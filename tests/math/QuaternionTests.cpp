#include <gtest/gtest.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>
#include <math/Matrix.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>

namespace aeroforge::math::tests
{
	namespace
	{
		constexpr double EPSILON = 1e-10;
		constexpr double PI = std::numbers::pi_v<double>;

		void expectQuaternionNear(
			const Quaternion& actual,
			const Quaternion& expected,
			double epsilon = EPSILON)
		{
			EXPECT_NEAR(actual.w, expected.w, epsilon);
			EXPECT_NEAR(actual.x, expected.x, epsilon);
			EXPECT_NEAR(actual.y, expected.y, epsilon);
			EXPECT_NEAR(actual.z, expected.z, epsilon);
		}

		void expectVectorNear(
			const Vector3& actual,
			const Vector3& expected,
			double epsilon = EPSILON)
		{
			EXPECT_NEAR(actual.x, expected.x, epsilon);
			EXPECT_NEAR(actual.y, expected.y, epsilon);
			EXPECT_NEAR(actual.z, expected.z, epsilon);
		}

		void expectMatrixNear(
			const Matrix& actual,
			const Matrix& expected,
			double epsilon = EPSILON)
		{
			ASSERT_EQ(actual.rows(), expected.rows());
			ASSERT_EQ(actual.columns(), expected.columns());

			for (std::size_t row = 0;
				row < actual.rows();
				++row)
			{
				for (std::size_t column = 0;
					column < actual.columns();
					++column)
				{
					EXPECT_NEAR(
						actual(row, column),
						expected(row, column),
						epsilon);
				}
			}
		}

		void expectSameRotation(
			const Quaternion& actual,
			const Quaternion& expected,
			double epsilon = EPSILON)
		{
			EXPECT_TRUE(
				actual.isSameRotation(
					expected,
					epsilon));
		}
	}

	// ============================================================
	// Construction
	// ============================================================

	TEST(QuaternionConstructionTests, DefaultConstructorCreatesIdentity)
	{
		const Quaternion q;

		EXPECT_DOUBLE_EQ(q.w, 1.0);
		EXPECT_DOUBLE_EQ(q.x, 0.0);
		EXPECT_DOUBLE_EQ(q.y, 0.0);
		EXPECT_DOUBLE_EQ(q.z, 0.0);
	}

	TEST(QuaternionConstructionTests, ExplicitConstructorStoresComponents)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		EXPECT_DOUBLE_EQ(q.w, 1.0);
		EXPECT_DOUBLE_EQ(q.x, 2.0);
		EXPECT_DOUBLE_EQ(q.y, 3.0);
		EXPECT_DOUBLE_EQ(q.z, 4.0);
	}

	TEST(QuaternionConstructionTests, IdentityFactoryCreatesIdentity)
	{
		const Quaternion q =
			Quaternion::identity();

		expectQuaternionNear(
			q,
			Quaternion(
				1.0,
				0.0,
				0.0,
				0.0));
	}

	// ============================================================
	// Indexing
	// ============================================================

	TEST(QuaternionIndexTests, MutableIndexReturnsCorrectComponents)
	{
		Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		q[0] = 5.0;
		q[1] = 6.0;
		q[2] = 7.0;
		q[3] = 8.0;

		EXPECT_DOUBLE_EQ(q.w, 5.0);
		EXPECT_DOUBLE_EQ(q.x, 6.0);
		EXPECT_DOUBLE_EQ(q.y, 7.0);
		EXPECT_DOUBLE_EQ(q.z, 8.0);
	}

	TEST(QuaternionIndexTests, ConstIndexReturnsCorrectComponents)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		EXPECT_DOUBLE_EQ(q[0], 1.0);
		EXPECT_DOUBLE_EQ(q[1], 2.0);
		EXPECT_DOUBLE_EQ(q[2], 3.0);
		EXPECT_DOUBLE_EQ(q[3], 4.0);
	}

	TEST(QuaternionIndexTests, MutableIndexRejectsOutOfRange)
	{
		Quaternion q;

		EXPECT_THROW(
			q[4],
			std::out_of_range);

		EXPECT_THROW(
			q[100],
			std::out_of_range);
	}

	TEST(QuaternionIndexTests, ConstIndexRejectsOutOfRange)
	{
		const Quaternion q;

		EXPECT_THROW(
			q[4],
			std::out_of_range);
	}

	// ============================================================
	// Addition
	// ============================================================

	TEST(QuaternionArithmeticTests, Add)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			5.0,
			6.0,
			7.0,
			8.0);

		expectQuaternionNear(
			a.add(b),
			Quaternion(
				6.0,
				8.0,
				10.0,
				12.0));
	}

	TEST(QuaternionArithmeticTests, AdditionOperator)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			5.0,
			6.0,
			7.0,
			8.0);

		expectQuaternionNear(
			a + b,
			Quaternion(
				6.0,
				8.0,
				10.0,
				12.0));
	}

	TEST(QuaternionArithmeticTests, AdditionAssignmentOperator)
	{
		Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion other(
			5.0,
			6.0,
			7.0,
			8.0);

		Quaternion& returned =
			(q += other);

		EXPECT_EQ(&returned, &q);

		expectQuaternionNear(
			q,
			Quaternion(
				6.0,
				8.0,
				10.0,
				12.0));
	}

	// ============================================================
	// Subtraction
	// ============================================================

	TEST(QuaternionArithmeticTests, Subtract)
	{
		const Quaternion a(
			5.0,
			7.0,
			9.0,
			11.0);

		const Quaternion b(
			1.0,
			2.0,
			3.0,
			4.0);

		expectQuaternionNear(
			a.sub(b),
			Quaternion(
				4.0,
				5.0,
				6.0,
				7.0));
	}

	TEST(QuaternionArithmeticTests, SubtractionOperator)
	{
		const Quaternion a(
			5.0,
			7.0,
			9.0,
			11.0);

		const Quaternion b(
			1.0,
			2.0,
			3.0,
			4.0);

		expectQuaternionNear(
			a - b,
			Quaternion(
				4.0,
				5.0,
				6.0,
				7.0));
	}

	TEST(QuaternionArithmeticTests, SubtractionAssignmentOperator)
	{
		Quaternion q(
			5.0,
			7.0,
			9.0,
			11.0);

		q -= Quaternion(
			1.0,
			2.0,
			3.0,
			4.0);

		expectQuaternionNear(
			q,
			Quaternion(
				4.0,
				5.0,
				6.0,
				7.0));
	}

	// ============================================================
	// Scalar operations
	// ============================================================

	TEST(QuaternionScalarTests, Scale)
	{
		const Quaternion q(
			1.0,
			-2.0,
			3.0,
			-4.0);

		expectQuaternionNear(
			q.scale(2.0),
			Quaternion(
				2.0,
				-4.0,
				6.0,
				-8.0));
	}

	TEST(QuaternionScalarTests, QuaternionTimesScalar)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		expectQuaternionNear(
			q * 3.0,
			Quaternion(
				3.0,
				6.0,
				9.0,
				12.0));
	}

	TEST(QuaternionScalarTests, ScalarTimesQuaternion)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		expectQuaternionNear(
			3.0 * q,
			Quaternion(
				3.0,
				6.0,
				9.0,
				12.0));
	}

	TEST(QuaternionScalarTests, ScalarMultiplyAssignment)
	{
		Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		q *= 2.0;

		expectQuaternionNear(
			q,
			Quaternion(
				2.0,
				4.0,
				6.0,
				8.0));
	}

	TEST(QuaternionScalarTests, Divide)
	{
		const Quaternion q(
			2.0,
			4.0,
			6.0,
			8.0);

		expectQuaternionNear(
			q.div(2.0),
			Quaternion(
				1.0,
				2.0,
				3.0,
				4.0));
	}

	TEST(QuaternionScalarTests, DivisionOperator)
	{
		const Quaternion q(
			2.0,
			4.0,
			6.0,
			8.0);

		expectQuaternionNear(
			q / 2.0,
			Quaternion(
				1.0,
				2.0,
				3.0,
				4.0));
	}

	TEST(QuaternionScalarTests, DivisionAssignment)
	{
		Quaternion q(
			2.0,
			4.0,
			6.0,
			8.0);

		q /= 2.0;

		expectQuaternionNear(
			q,
			Quaternion(
				1.0,
				2.0,
				3.0,
				4.0));
	}

	TEST(QuaternionScalarTests, DivideRejectsZero)
	{
		const Quaternion q;

		EXPECT_THROW(
			q.div(0.0),
			std::invalid_argument);
	}

	TEST(QuaternionScalarTests, DivisionOperatorRejectsZero)
	{
		const Quaternion q;

		EXPECT_THROW(
			q / 0.0,
			std::invalid_argument);
	}

	TEST(QuaternionScalarTests, DivisionAssignmentRejectsZero)
	{
		Quaternion q;

		EXPECT_THROW(
			q /= 0.0,
			std::invalid_argument);
	}

	// ============================================================
	// Dot / norm
	// ============================================================

	TEST(QuaternionNormTests, DotProduct)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			5.0,
			6.0,
			7.0,
			8.0);

		EXPECT_DOUBLE_EQ(
			a.dot(b),
			70.0);
	}

	TEST(QuaternionNormTests, SquaredNorm)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		EXPECT_DOUBLE_EQ(
			q.squaredNorm(),
			30.0);
	}

	TEST(QuaternionNormTests, Norm)
	{
		const Quaternion q(
			1.0,
			2.0,
			2.0,
			0.0);

		EXPECT_DOUBLE_EQ(
			q.norm(),
			3.0);
	}

	TEST(QuaternionNormTests, NormalizeModifiesQuaternion)
	{
		Quaternion q(
			2.0,
			0.0,
			0.0,
			0.0);

		Quaternion& returned =
			q.normalize();

		EXPECT_EQ(&returned, &q);

		expectQuaternionNear(
			q,
			Quaternion::identity());

		EXPECT_NEAR(
			q.norm(),
			1.0,
			EPSILON);
	}

	TEST(QuaternionNormTests, NormalizedDoesNotModifyOriginal)
	{
		const Quaternion q(
			2.0,
			0.0,
			0.0,
			0.0);

		const Quaternion result =
			q.normalized();

		EXPECT_DOUBLE_EQ(q.w, 2.0);

		expectQuaternionNear(
			result,
			Quaternion::identity());
	}

	TEST(QuaternionNormTests, NormalizeRejectsZeroQuaternion)
	{
		Quaternion q(
			0.0,
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			q.normalize(),
			std::domain_error);
	}

	TEST(QuaternionNormTests, NormalizedRejectsZeroQuaternion)
	{
		const Quaternion q(
			0.0,
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			q.normalized(),
			std::domain_error);
	}

	// ============================================================
	// Conjugate / inverse
	// ============================================================

	TEST(QuaternionAlgebraTests, Conjugate)
	{
		const Quaternion q(
			1.0,
			2.0,
			-3.0,
			4.0);

		expectQuaternionNear(
			q.conjugate(),
			Quaternion(
				1.0,
				-2.0,
				3.0,
				-4.0));
	}

	TEST(QuaternionAlgebraTests, ConjugateProductEqualsSquaredNorm)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion result =
			q * q.conjugate();

		expectQuaternionNear(
			result,
			Quaternion(
				q.squaredNorm(),
				0.0,
				0.0,
				0.0));
	}

	TEST(QuaternionAlgebraTests, Inverse)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion result =
			q * q.inverse();

		expectQuaternionNear(
			result,
			Quaternion::identity(),
			1e-9);
	}

	TEST(QuaternionAlgebraTests, InverseWorksOnBothSides)
	{
		const Quaternion q(
			1.0,
			-2.0,
			0.5,
			3.0);

		expectQuaternionNear(
			q * q.inverse(),
			Quaternion::identity(),
			1e-9);

		expectQuaternionNear(
			q.inverse() * q,
			Quaternion::identity(),
			1e-9);
	}

	TEST(QuaternionAlgebraTests, InverseRejectsZeroQuaternion)
	{
		const Quaternion q(
			0.0,
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			q.inverse(),
			std::domain_error);
	}

	// ============================================================
	// Hamilton product
	// ============================================================

	TEST(QuaternionMultiplicationTests, HamiltonProductKnownValues)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			5.0,
			6.0,
			7.0,
			8.0);

		expectQuaternionNear(
			a.mult(b),
			Quaternion(
				-60.0,
				12.0,
				30.0,
				24.0));
	}

	TEST(QuaternionMultiplicationTests, OperatorMatchesMult)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			-2.0,
			0.5,
			4.0,
			1.0);

		expectQuaternionNear(
			a * b,
			a.mult(b));
	}

	TEST(QuaternionMultiplicationTests, IdentityOnRight)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		expectQuaternionNear(
			q * Quaternion::identity(),
			q);
	}

	TEST(QuaternionMultiplicationTests, IdentityOnLeft)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		expectQuaternionNear(
			Quaternion::identity() * q,
			q);
	}

	TEST(QuaternionMultiplicationTests, MultiplicationIsNotCommutative)
	{
		const Quaternion xRotation =
			Quaternion::fromRotationX(
				PI / 2.0);

		const Quaternion yRotation =
			Quaternion::fromRotationY(
				PI / 2.0);

		const Quaternion xy =
			xRotation * yRotation;

		const Quaternion yx =
			yRotation * xRotation;

		EXPECT_FALSE(
			xy.isSameRotation(
				yx,
				EPSILON));
	}

	TEST(QuaternionMultiplicationTests, MultiplyAssignmentMatchesOperator)
	{
		Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			0.5,
			-1.0,
			2.0,
			3.0);

		const Quaternion expected =
			a * b;

		a *= b;

		expectQuaternionNear(
			a,
			expected);
	}

	// ============================================================
	// Axis-angle construction
	// ============================================================

	TEST(QuaternionAxisAngleTests, ZeroAngleCreatesIdentityRotation)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3::unitX(),
				0.0);

		expectSameRotation(
			q,
			Quaternion::identity());
	}

	TEST(QuaternionAxisAngleTests, NinetyDegreesAroundX)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3::unitX(),
				PI / 2.0);

		const double expected =
			std::sqrt(0.5);

		expectQuaternionNear(
			q,
			Quaternion(
				expected,
				expected,
				0.0,
				0.0));
	}

	TEST(QuaternionAxisAngleTests, NinetyDegreesAroundY)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3::unitY(),
				PI / 2.0);

		const double expected =
			std::sqrt(0.5);

		expectQuaternionNear(
			q,
			Quaternion(
				expected,
				0.0,
				expected,
				0.0));
	}

	TEST(QuaternionAxisAngleTests, NinetyDegreesAroundZ)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3::unitZ(),
				PI / 2.0);

		const double expected =
			std::sqrt(0.5);

		expectQuaternionNear(
			q,
			Quaternion(
				expected,
				0.0,
				0.0,
				expected));
	}

	TEST(QuaternionAxisAngleTests, NormalizesInputAxis)
	{
		const Quaternion unitAxis =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					0.0,
					0.0),
				0.8);

		const Quaternion scaledAxis =
			Quaternion::fromAxisAngle(
				Vector3(
					10.0,
					0.0,
					0.0),
				0.8);

		expectSameRotation(
			unitAxis,
			scaledAxis);
	}

	TEST(QuaternionAxisAngleTests, RejectsZeroAxis)
	{
		EXPECT_THROW(
			Quaternion::fromAxisAngle(
				Vector3(
					0.0,
					0.0,
					0.0),
				PI / 2.0),
			std::domain_error);
	}

	TEST(QuaternionAxisAngleTests, ResultIsNormalized)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					2.0,
					3.0),
				1.234);

		EXPECT_TRUE(
			q.isNormalized(
				EPSILON));
	}

	// ============================================================
	// Rotation axis / angle extraction
	// ============================================================

	TEST(QuaternionAxisExtractionTests, ExtractsRotationAxis)
	{
		const Vector3 axis(
			1.0,
			2.0,
			3.0);

		const Quaternion q =
			Quaternion::fromAxisAngle(
				axis,
				1.2);

		const Vector3 expectedAxis =
			axis / axis.magnitude();

		expectVectorNear(
			q.rotationAxis(),
			expectedAxis);
	}

	TEST(QuaternionAxisExtractionTests, ExtractsRotationAngle)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3::unitY(),
				1.234);

		EXPECT_NEAR(
			q.rotationAngle(),
			1.234,
			EPSILON);
	}

	TEST(QuaternionAxisExtractionTests, IdentityUsesUnitXAsConventionalAxis)
	{
		expectVectorNear(
			Quaternion::identity()
			.rotationAxis(),
			Vector3::unitX());
	}

	TEST(QuaternionAxisExtractionTests, IdentityRotationAngleIsZero)
	{
		EXPECT_NEAR(
			Quaternion::identity()
			.rotationAngle(),
			0.0,
			EPSILON);
	}

	TEST(QuaternionAxisExtractionTests, NegativeQuaternionHasSameAxisAndAngle)
	{
		const Quaternion original =
			Quaternion::fromAxisAngle(
				Vector3::unitZ(),
				0.8);

		const Quaternion negative =
			original * -1.0;

		expectVectorNear(
			negative.rotationAxis(),
			original.rotationAxis());

		EXPECT_NEAR(
			negative.rotationAngle(),
			original.rotationAngle(),
			EPSILON);
	}

	TEST(QuaternionAxisExtractionTests, UsesShortestEquivalentRotation)
	{
		const Quaternion q =
			Quaternion::fromRotationZ(
				3.0 * PI / 2.0);

		EXPECT_NEAR(
			q.rotationAngle(),
			PI / 2.0,
			EPSILON);

		expectVectorNear(
			q.rotationAxis(),
			Vector3(
				0.0,
				0.0,
				-1.0));
	}

	// ============================================================
	// Principal rotations
	// ============================================================

	TEST(QuaternionPrincipalRotationTests, RotationXMatchesAxisAngle)
	{
		const double angle = 0.73;

		expectSameRotation(
			Quaternion::fromRotationX(angle),
			Quaternion::fromAxisAngle(
				Vector3::unitX(),
				angle));
	}

	TEST(QuaternionPrincipalRotationTests, RotationYMatchesAxisAngle)
	{
		const double angle = 0.73;

		expectSameRotation(
			Quaternion::fromRotationY(angle),
			Quaternion::fromAxisAngle(
				Vector3::unitY(),
				angle));
	}

	TEST(QuaternionPrincipalRotationTests, RotationZMatchesAxisAngle)
	{
		const double angle = 0.73;

		expectSameRotation(
			Quaternion::fromRotationZ(angle),
			Quaternion::fromAxisAngle(
				Vector3::unitZ(),
				angle));
	}

	// ============================================================
	// Vector rotation
	// ============================================================

	TEST(QuaternionVectorRotationTests, XRotationMapsPositiveYToPositiveZ)
	{
		const Quaternion q =
			Quaternion::fromRotationX(
				PI / 2.0);

		expectVectorNear(
			q.rotate(
				Vector3(
					0.0,
					1.0,
					0.0)),
			Vector3(
				0.0,
				0.0,
				1.0));
	}

	TEST(QuaternionVectorRotationTests, YRotationMapsPositiveZToPositiveX)
	{
		const Quaternion q =
			Quaternion::fromRotationY(
				PI / 2.0);

		expectVectorNear(
			q.rotate(
				Vector3(
					0.0,
					0.0,
					1.0)),
			Vector3(
				1.0,
				0.0,
				0.0));
	}

	TEST(QuaternionVectorRotationTests, ZRotationMapsPositiveXToPositiveY)
	{
		const Quaternion q =
			Quaternion::fromRotationZ(
				PI / 2.0);

		expectVectorNear(
			q.rotate(
				Vector3(
					1.0,
					0.0,
					0.0)),
			Vector3(
				0.0,
				1.0,
				0.0));
	}

	TEST(QuaternionVectorRotationTests, RotationPreservesVectorMagnitude)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					2.0,
					3.0),
				1.37);

		const Vector3 vector(
			4.0,
			-8.0,
			2.5);

		const Vector3 rotated =
			q.rotate(vector);

		EXPECT_NEAR(
			rotated.magnitude(),
			vector.magnitude(),
			EPSILON);
	}

	TEST(QuaternionVectorRotationTests, RotateInverseRestoresOriginalVector)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					2.0,
					3.0),
				1.1);

		const Vector3 original(
			7.0,
			-2.0,
			5.0);

		const Vector3 rotated =
			q.rotate(original);

		const Vector3 restored =
			q.rotateInverse(rotated);

		expectVectorNear(
			restored,
			original,
			1e-9);
	}

	TEST(QuaternionVectorRotationTests, NonNormalizedQuaternionStillRotatesCorrectly)
	{
		const Quaternion normalized =
			Quaternion::fromRotationZ(
				PI / 2.0);

		const Quaternion scaled =
			normalized * 10.0;

		const Vector3 vector(
			1.0,
			0.0,
			0.0);

		expectVectorNear(
			scaled.rotate(vector),
			normalized.rotate(vector));
	}

	TEST(QuaternionVectorRotationTests, ZeroQuaternionCannotRotateVector)
	{
		const Quaternion zero(
			0.0,
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			zero.rotate(
				Vector3::unitX()),
			std::domain_error);
	}

	// ============================================================
	// Rotation matrix conversion
	// ============================================================

	TEST(QuaternionMatrixTests, IdentityProducesIdentityMatrix)
	{
		const Matrix result =
			Quaternion::identity()
			.toRotationMatrix();

		expectMatrixNear(
			result,
			Matrix::identity(3));
	}

	TEST(QuaternionMatrixTests, RotationXProducesExpectedMatrix)
	{
		const Matrix result =
			Quaternion::fromRotationX(
				PI / 2.0)
			.toRotationMatrix();

		const Matrix expected{
			{1.0, 0.0, 0.0},
			{0.0, 0.0, -1.0},
			{0.0, 1.0, 0.0}
		};

		expectMatrixNear(
			result,
			expected);
	}

	TEST(QuaternionMatrixTests, RotationYProducesExpectedMatrix)
	{
		const Matrix result =
			Quaternion::fromRotationY(
				PI / 2.0)
			.toRotationMatrix();

		const Matrix expected{
			{0.0, 0.0, 1.0},
			{0.0, 1.0, 0.0},
			{-1.0, 0.0, 0.0}
		};

		expectMatrixNear(
			result,
			expected);
	}

	TEST(QuaternionMatrixTests, RotationZProducesExpectedMatrix)
	{
		const Matrix result =
			Quaternion::fromRotationZ(
				PI / 2.0)
			.toRotationMatrix();

		const Matrix expected{
			{0.0, -1.0, 0.0},
			{1.0, 0.0, 0.0},
			{0.0, 0.0, 1.0}
		};

		expectMatrixNear(
			result,
			expected);
	}

	TEST(QuaternionMatrixTests, RotationMatrixIsThreeByThree)
	{
		const Matrix matrix =
			Quaternion::fromRotationX(
				0.5)
			.toRotationMatrix();

		EXPECT_EQ(matrix.rows(), 3U);
		EXPECT_EQ(matrix.columns(), 3U);
	}

	TEST(QuaternionMatrixTests, RotationMatrixIsOrthogonal)
	{
		const Matrix matrix =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					2.0,
					3.0),
				1.4)
			.toRotationMatrix();

		const Matrix result =
			matrix.transpose() * matrix;

		expectMatrixNear(
			result,
			Matrix::identity(3),
			1e-9);
	}

	TEST(QuaternionMatrixTests, RotationMatrixDeterminantIsOne)
	{
		const Matrix matrix =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					2.0,
					3.0),
				1.4)
			.toRotationMatrix();

		EXPECT_NEAR(
			matrix.determinant(),
			1.0,
			1e-9);
	}

	TEST(QuaternionMatrixTests, QuaternionRotationMatchesMatrixRotation)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					2.0,
					3.0),
				0.91);

		const Vector3 vector(
			4.0,
			-2.0,
			7.0);

		const Vector3 quaternionResult =
			q.rotate(vector);

		const Vector3 matrixResult =
			q.toRotationMatrix() * vector;

		expectVectorNear(
			quaternionResult,
			matrixResult,
			1e-9);
	}

	// ============================================================
	// Matrix -> Quaternion
	// ============================================================

	TEST(QuaternionFromMatrixTests, IdentityMatrixProducesIdentity)
	{
		const Quaternion result =
			Quaternion::fromRotationMatrix(
				Matrix::identity(3));

		expectSameRotation(
			result,
			Quaternion::identity());
	}

	TEST(QuaternionFromMatrixTests, RotationXRoundTrip)
	{
		const Quaternion original =
			Quaternion::fromRotationX(
				PI);

		const Quaternion recovered =
			Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original,
			1e-9);
	}

	TEST(QuaternionFromMatrixTests, RotationYRoundTrip)
	{
		const Quaternion original =
			Quaternion::fromRotationY(
				PI);

		const Quaternion recovered =
			Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original,
			1e-9);
	}

	TEST(QuaternionFromMatrixTests, RotationZRoundTrip)
	{
		const Quaternion original =
			Quaternion::fromRotationZ(
				PI);

		const Quaternion recovered =
			Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original,
			1e-9);
	}

	TEST(QuaternionFromMatrixTests, ArbitraryRotationRoundTrip)
	{
		const Quaternion original =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					-4.0,
					2.0),
				1.73);

		const Quaternion recovered =
			Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original,
			1e-9);
	}

	TEST(QuaternionFromMatrixTests, RejectsWrongDimensions)
	{
		const Matrix matrix(
			4,
			4);

		EXPECT_THROW(
			Quaternion::fromRotationMatrix(
				matrix),
			std::invalid_argument);
	}

	TEST(QuaternionFromMatrixTests, RejectsNonFiniteMatrix)
	{
		Matrix matrix =
			Matrix::identity(3);

		matrix(0, 0) =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			Quaternion::fromRotationMatrix(
				matrix),
			std::invalid_argument);
	}

	TEST(QuaternionFromMatrixTests, RejectsScalingMatrix)
	{
		const Matrix matrix{
			{2.0, 0.0, 0.0},
			{0.0, 1.0, 0.0},
			{0.0, 0.0, 1.0}
		};

		EXPECT_THROW(
			Quaternion::fromRotationMatrix(
				matrix),
			std::invalid_argument);
	}

	TEST(QuaternionFromMatrixTests, RejectsReflectionMatrix)
	{
		const Matrix matrix{
			{1.0, 0.0, 0.0},
			{0.0, 1.0, 0.0},
			{0.0, 0.0, -1.0}
		};

		EXPECT_THROW(
			Quaternion::fromRotationMatrix(
				matrix),
			std::invalid_argument);
	}

	// ============================================================
	// fromTwoVectors
	// ============================================================

	TEST(QuaternionFromTwoVectorsTests, SameDirectionReturnsIdentity)
	{
		const Quaternion q =
			Quaternion::fromTwoVectors(
				Vector3::unitX(),
				Vector3::unitX());

		EXPECT_TRUE(
			q.isIdentity(
				EPSILON));
	}

	TEST(QuaternionFromTwoVectorsTests, PositiveXToPositiveY)
	{
		const Vector3 from =
			Vector3::unitX();

		const Vector3 to =
			Vector3::unitY();

		const Quaternion q =
			Quaternion::fromTwoVectors(
				from,
				to);

		expectVectorNear(
			q.rotate(from),
			to);
	}

	TEST(QuaternionFromTwoVectorsTests, PositiveYToPositiveZ)
	{
		const Vector3 from =
			Vector3::unitY();

		const Vector3 to =
			Vector3::unitZ();

		const Quaternion q =
			Quaternion::fromTwoVectors(
				from,
				to);

		expectVectorNear(
			q.rotate(from),
			to);
	}

	TEST(QuaternionFromTwoVectorsTests, OppositeDirections)
	{
		const Vector3 from(
			1.0,
			0.0,
			0.0);

		const Vector3 to(
			-1.0,
			0.0,
			0.0);

		const Quaternion q =
			Quaternion::fromTwoVectors(
				from,
				to);

		expectVectorNear(
			q.rotate(from),
			to,
			1e-9);

		EXPECT_NEAR(
			q.rotationAngle(),
			PI,
			1e-9);
	}

	TEST(QuaternionFromTwoVectorsTests, HandlesNonNormalizedVectors)
	{
		const Vector3 from(
			10.0,
			0.0,
			0.0);

		const Vector3 to(
			0.0,
			-25.0,
			0.0);

		const Quaternion q =
			Quaternion::fromTwoVectors(
				from,
				to);

		const Vector3 expected =
			to / to.magnitude();

		const Vector3 rotated =
			q.rotate(from);

		const Vector3 actual =
			rotated / rotated.magnitude();

		expectVectorNear(
			actual,
			expected);
	}

	TEST(QuaternionFromTwoVectorsTests, ArbitraryDirections)
	{
		const Vector3 from(
			1.0,
			2.0,
			3.0);

		const Vector3 to(
			-4.0,
			5.0,
			1.0);

		const Quaternion q =
			Quaternion::fromTwoVectors(
				from,
				to);

		const Vector3 fromNormalized =
			from / from.magnitude();

		const Vector3 toNormalized =
			to / to.magnitude();

		expectVectorNear(
			q.rotate(fromNormalized),
			toNormalized,
			1e-9);
	}

	TEST(QuaternionFromTwoVectorsTests, RejectsZeroFromVector)
	{
		EXPECT_THROW(
			Quaternion::fromTwoVectors(
				Vector3(
					0.0,
					0.0,
					0.0),
				Vector3::unitX()),
			std::domain_error);
	}

	TEST(QuaternionFromTwoVectorsTests, RejectsZeroToVector)
	{
		EXPECT_THROW(
			Quaternion::fromTwoVectors(
				Vector3::unitX(),
				Vector3(
					0.0,
					0.0,
					0.0)),
			std::domain_error);
	}

	// ============================================================
	// Numerical safety
	// ============================================================

	TEST(QuaternionNumericalTests, FiniteQuaternionIsFinite)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		EXPECT_TRUE(
			q.isFinite());
	}

	TEST(QuaternionNumericalTests, NaNQuaternionIsNotFinite)
	{
		const Quaternion q(
			std::numeric_limits<double>::quiet_NaN(),
			0.0,
			0.0,
			0.0);

		EXPECT_FALSE(
			q.isFinite());
	}

	TEST(QuaternionNumericalTests, InfiniteQuaternionIsNotFinite)
	{
		const Quaternion q(
			1.0,
			std::numeric_limits<double>::infinity(),
			0.0,
			0.0);

		EXPECT_FALSE(
			q.isFinite());
	}

	TEST(QuaternionNumericalTests, IsApproxIdentical)
	{
		const Quaternion q(
			1.0,
			2.0,
			3.0,
			4.0);

		EXPECT_TRUE(
			q.isApprox(
				q,
				EPSILON));
	}

	TEST(QuaternionNumericalTests, IsApproxSmallDifference)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			1.0 + 1e-12,
			2.0,
			3.0,
			4.0);

		EXPECT_TRUE(
			a.isApprox(
				b,
				1e-9));
	}

	TEST(QuaternionNumericalTests, IsApproxRejectsLargeDifference)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			1.1,
			2.0,
			3.0,
			4.0);

		EXPECT_FALSE(
			a.isApprox(
				b,
				1e-6));
	}

	TEST(QuaternionNumericalTests, IsApproxSupportsRelativeTolerance)
	{
		const Quaternion a(
			1e12,
			2e12,
			3e12,
			4e12);

		const Quaternion b(
			1e12 + 100.0,
			2e12,
			3e12,
			4e12);

		EXPECT_TRUE(
			a.isApprox(
				b,
				1e-9));
	}

	TEST(QuaternionNumericalTests, IsApproxRejectsNonPositiveEpsilon)
	{
		const Quaternion q;

		EXPECT_THROW(
			q.isApprox(q, 0.0),
			std::invalid_argument);

		EXPECT_THROW(
			q.isApprox(q, -1.0),
			std::invalid_argument);
	}

	TEST(QuaternionNumericalTests, IsApproxRejectsNaNValues)
	{
		const Quaternion a;

		const Quaternion b(
			std::numeric_limits<double>::quiet_NaN(),
			0.0,
			0.0,
			0.0);

		EXPECT_FALSE(
			a.isApprox(
				b,
				EPSILON));
	}

	TEST(QuaternionNumericalTests, IdentityIsNormalized)
	{
		EXPECT_TRUE(
			Quaternion::identity()
			.isNormalized(EPSILON));
	}

	TEST(QuaternionNumericalTests, ScaledQuaternionIsNotNormalized)
	{
		const Quaternion q(
			2.0,
			0.0,
			0.0,
			0.0);

		EXPECT_FALSE(
			q.isNormalized(
				EPSILON));
	}

	TEST(QuaternionNumericalTests, IsNormalizedRejectsInvalidEpsilon)
	{
		EXPECT_THROW(
			Quaternion::identity()
			.isNormalized(0.0),
			std::invalid_argument);
	}

	TEST(QuaternionNumericalTests, IdentityRecognizesScaledEquivalentQuaternion)
	{
		const Quaternion q(
			5.0,
			0.0,
			0.0,
			0.0);

		EXPECT_TRUE(
			q.isIdentity(
				EPSILON));
	}

	TEST(QuaternionNumericalTests, NegativeIdentityRepresentsIdentityRotation)
	{
		const Quaternion q(
			-1.0,
			0.0,
			0.0,
			0.0);

		EXPECT_TRUE(
			q.isIdentity(
				EPSILON));
	}

	TEST(QuaternionNumericalTests, SameRotationAcceptsNegativeQuaternion)
	{
		const Quaternion q =
			Quaternion::fromRotationY(
				0.8);

		const Quaternion negative =
			q * -1.0;

		EXPECT_TRUE(
			q.isSameRotation(
				negative,
				EPSILON));
	}

	TEST(QuaternionNumericalTests, SameRotationAcceptsScaledQuaternion)
	{
		const Quaternion q =
			Quaternion::fromRotationY(
				0.8);

		const Quaternion scaled =
			q * 10.0;

		EXPECT_TRUE(
			q.isSameRotation(
				scaled,
				EPSILON));
	}

	TEST(QuaternionNumericalTests, DifferentRotationsAreNotSame)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.5);

		const Quaternion b =
			Quaternion::fromRotationY(
				0.5);

		EXPECT_FALSE(
			a.isSameRotation(
				b,
				EPSILON));
	}

	TEST(QuaternionNumericalTests, SameRotationRejectsZeroQuaternion)
	{
		const Quaternion zero(
			0.0,
			0.0,
			0.0,
			0.0);

		EXPECT_FALSE(
			zero.isSameRotation(
				Quaternion::identity(),
				EPSILON));
	}

	// ============================================================
	// LERP
	// ============================================================

	TEST(QuaternionLerpTests, TZeroReturnsFirstQuaternion)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			5.0,
			6.0,
			7.0,
			8.0);

		expectQuaternionNear(
			a.lerp(b, 0.0),
			a);
	}

	TEST(QuaternionLerpTests, TOneReturnsSecondQuaternion)
	{
		const Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const Quaternion b(
			5.0,
			6.0,
			7.0,
			8.0);

		expectQuaternionNear(
			a.lerp(b, 1.0),
			b);
	}

	TEST(QuaternionLerpTests, HalfwayInterpolatesComponents)
	{
		const Quaternion a(
			1.0,
			0.0,
			0.0,
			0.0);

		const Quaternion b(
			0.0,
			1.0,
			0.0,
			0.0);

		expectQuaternionNear(
			a.lerp(b, 0.5),
			Quaternion(
				0.5,
				0.5,
				0.0,
				0.0));
	}

	TEST(QuaternionLerpTests, DoesNotAutomaticallyNormalize)
	{
		const Quaternion a =
			Quaternion::identity();

		const Quaternion b =
			Quaternion::fromRotationX(PI);

		const Quaternion result =
			a.lerp(b, 0.5);

		EXPECT_FALSE(
			result.isNormalized(
				1e-6));
	}

	// ============================================================
	// NLERP
	// ============================================================

	TEST(QuaternionNlerpTests, TZeroReturnsFirstRotation)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.2);

		const Quaternion b =
			Quaternion::fromRotationX(
				1.0);

		expectSameRotation(
			a.nlerp(b, 0.0),
			a);
	}

	TEST(QuaternionNlerpTests, TOneReturnsSecondRotation)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.2);

		const Quaternion b =
			Quaternion::fromRotationX(
				1.0);

		expectSameRotation(
			a.nlerp(b, 1.0),
			b);
	}

	TEST(QuaternionNlerpTests, ResultIsNormalized)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.2);

		const Quaternion b =
			Quaternion::fromRotationY(
				1.0);

		EXPECT_TRUE(
			a.nlerp(
				b,
				0.37)
			.isNormalized(EPSILON));
	}

	TEST(QuaternionNlerpTests, UsesShortestQuaternionPath)
	{
		const Quaternion q =
			Quaternion::fromRotationZ(
				1.0);

		const Quaternion negative =
			q * -1.0;

		const Quaternion result =
			q.nlerp(
				negative,
				0.5);

		expectSameRotation(
			result,
			q);
	}

	// ============================================================
	// SLERP
	// ============================================================

	TEST(QuaternionSlerpTests, TZeroReturnsFirstRotation)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.3);

		const Quaternion b =
			Quaternion::fromRotationY(
				1.1);

		expectSameRotation(
			a.slerp(b, 0.0),
			a);
	}

	TEST(QuaternionSlerpTests, TOneReturnsSecondRotation)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.3);

		const Quaternion b =
			Quaternion::fromRotationY(
				1.1);

		expectSameRotation(
			a.slerp(b, 1.0),
			b);
	}

	TEST(QuaternionSlerpTests, HalfwayBetweenIdentityAndNinetyDegreesIsFortyFiveDegrees)
	{
		const Quaternion a =
			Quaternion::identity();

		const Quaternion b =
			Quaternion::fromRotationZ(
				PI / 2.0);

		const Quaternion result =
			a.slerp(
				b,
				0.5);

		const Quaternion expected =
			Quaternion::fromRotationZ(
				PI / 4.0);

		expectSameRotation(
			result,
			expected,
			1e-9);
	}

	TEST(QuaternionSlerpTests, ResultIsNormalized)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.4);

		const Quaternion b =
			Quaternion::fromRotationY(
				1.3);

		EXPECT_TRUE(
			a.slerp(
				b,
				0.42)
			.isNormalized(EPSILON));
	}

	TEST(QuaternionSlerpTests, HandlesNegativeEquivalentQuaternion)
	{
		const Quaternion q =
			Quaternion::fromRotationX(
				0.75);

		const Quaternion negative =
			q * -1.0;

		const Quaternion result =
			q.slerp(
				negative,
				0.5);

		expectSameRotation(
			result,
			q);
	}

	TEST(QuaternionSlerpTests, HandlesNearlyIdenticalQuaternions)
	{
		const Quaternion a =
			Quaternion::fromRotationX(
				0.5000000);

		const Quaternion b =
			Quaternion::fromRotationX(
				0.5000001);

		const Quaternion result =
			a.slerp(
				b,
				0.5);

		EXPECT_TRUE(
			result.isNormalized(
				EPSILON));

		EXPECT_TRUE(
			result.isFinite());
	}

	// ============================================================
	// Angular velocity derivative
	// ============================================================

	TEST(QuaternionDerivativeTests, IdentityDerivativeMatchesBodyAngularVelocity)
	{
		const Quaternion q =
			Quaternion::identity();

		const Vector3 omega(
			2.0,
			4.0,
			6.0);

		const Quaternion derivative =
			q.derivativeFromAngularVelocity(
				omega);

		expectQuaternionNear(
			derivative,
			Quaternion(
				0.0,
				1.0,
				2.0,
				3.0));
	}

	TEST(QuaternionDerivativeTests, ZeroAngularVelocityProducesZeroDerivative)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					2.0,
					3.0),
				0.8);

		const Quaternion derivative =
			q.derivativeFromAngularVelocity(
				Vector3(
					0.0,
					0.0,
					0.0));

		expectQuaternionNear(
			derivative,
			Quaternion(
				0.0,
				0.0,
				0.0,
				0.0));
	}

	TEST(QuaternionDerivativeTests, DerivativeMatchesSmallBodyFrameRotation)
	{
		const Quaternion q =
			Quaternion::fromAxisAngle(
				Vector3(
					1.0,
					-2.0,
					0.5),
				0.7);

		const Vector3 omega(
			0.2,
			-0.3,
			0.5);

		constexpr double dt =
			1e-6;

		const Quaternion derivative =
			q.derivativeFromAngularVelocity(
				omega);

		const Quaternion integrated =
			(q + derivative * dt)
			.normalized();

		Vector3 axis = omega;
		const double angularSpeed =
			axis.magnitude();

		axis.normalize();

		const Quaternion exactIncrement =
			Quaternion::fromAxisAngle(
				axis,
				angularSpeed * dt);

		/*
		 * omega is expressed in BODY coordinates,
		 * so the incremental rotation is applied
		 * on the right of q_NB.
		 */
		const Quaternion expected =
			(q * exactIncrement)
			.normalized();

		expectSameRotation(
			integrated,
			expected,
			1e-9);
	}

	// ============================================================
	// Composition
	// ============================================================

	TEST(QuaternionCompositionTests, QuaternionCompositionMatchesMatrixComposition)
	{
		const Quaternion qx =
			Quaternion::fromRotationX(
				0.4);

		const Quaternion qy =
			Quaternion::fromRotationY(
				-0.8);

		const Quaternion qz =
			Quaternion::fromRotationZ(
				1.2);

		const Quaternion combined =
			qz * qy * qx;

		const Matrix expected =
			qz.toRotationMatrix() *
			qy.toRotationMatrix() *
			qx.toRotationMatrix();

		expectMatrixNear(
			combined.toRotationMatrix(),
			expected,
			1e-9);
	}

	TEST(QuaternionCompositionTests, RotationOrderMatchesColumnVectorConvention)
	{
		const Quaternion rotateX =
			Quaternion::fromRotationX(
				PI / 2.0);

		const Quaternion rotateZ =
			Quaternion::fromRotationZ(
				PI / 2.0);

		const Vector3 vector(
			0.0,
			1.0,
			0.0);

		/*
		 * qz * qx means qx is applied first,
		 * followed by qz.
		 */
		const Vector3 result =
			(rotateZ * rotateX)
			.rotate(vector);

		expectVectorNear(
			result,
			Vector3(
				0.0,
				0.0,
				1.0),
			1e-9);
	}

	// ============================================================
	// Deterministic randomized property testing
	// ============================================================

	TEST(QuaternionPropertyTests, RandomizedRotationProperties)
	{
		std::mt19937_64 generator(
			0xA3E0F04EULL);

		std::uniform_real_distribution<double>
			componentDistribution(
				-10.0,
				10.0);

		std::uniform_real_distribution<double>
			angleDistribution(
				-PI,
				PI);

		for (int iteration = 0;
			iteration < 100;
			++iteration)
		{
			Vector3 axis(
				componentDistribution(generator),
				componentDistribution(generator),
				componentDistribution(generator));

			if (axis.magnitude() < 1e-6)
			{
				axis =
					Vector3::unitX();
			}

			const double angle =
				angleDistribution(generator);

			const Quaternion q =
				Quaternion::fromAxisAngle(
					axis,
					angle);

			const Vector3 vector(
				componentDistribution(generator),
				componentDistribution(generator),
				componentDistribution(generator));

			// Quaternion remains unit length.
			EXPECT_TRUE(
				q.isNormalized(
					1e-9));

			const Matrix rotationMatrix =
				q.toRotationMatrix();

			// R^T R = I.
			expectMatrixNear(
				rotationMatrix.transpose() *
				rotationMatrix,
				Matrix::identity(3),
				1e-9);

			// det(R) = +1.
			EXPECT_NEAR(
				rotationMatrix.determinant(),
				1.0,
				1e-9);

			// Matrix round-trip preserves orientation.
			const Quaternion recovered =
				Quaternion::fromRotationMatrix(
					rotationMatrix);

			EXPECT_TRUE(
				q.isSameRotation(
					recovered,
					1e-9));

			// Quaternion and matrix rotations agree.
			const Vector3 quaternionRotation =
				q.rotate(vector);

			const Vector3 matrixRotation =
				rotationMatrix * vector;

			expectVectorNear(
				quaternionRotation,
				matrixRotation,
				1e-8);

			// Rotation preserves vector magnitude.
			EXPECT_NEAR(
				quaternionRotation.magnitude(),
				vector.magnitude(),
				1e-8);

			// Inverse rotation restores the input.
			const Vector3 restored =
				q.rotateInverse(
					quaternionRotation);

			expectVectorNear(
				restored,
				vector,
				1e-8);
		}
	}
}