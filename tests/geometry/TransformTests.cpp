#include <gtest/gtest.h>

#include <geometry/transform.h>
#include <math/Matrix.h>
#include <math/Vector3.h>

#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>

namespace aeroforge::geometry::tests
{
	namespace
	{
		constexpr double EPSILON = 1e-12;

		void expectMatrixNear(
			const aeroforge::math::Matrix& actual,
			const aeroforge::math::Matrix& expected,
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
						epsilon)
						<< "Mismatch at ("
						<< row << ", "
						<< column << ")";
				}
			}
		}

		void expectHomogeneousPointNear(
			const aeroforge::math::Matrix& actual,
			double x,
			double y,
			double z,
			double epsilon = EPSILON)
		{
			ASSERT_EQ(actual.rows(), 4);
			ASSERT_EQ(actual.columns(), 1);

			EXPECT_NEAR(actual(0, 0), x, epsilon);
			EXPECT_NEAR(actual(1, 0), y, epsilon);
			EXPECT_NEAR(actual(2, 0), z, epsilon);
			EXPECT_NEAR(actual(3, 0), 1.0, epsilon);
		}

		aeroforge::math::Matrix makePoint(
			double x,
			double y,
			double z)
		{
			return aeroforge::math::Matrix{
				{x},
				{y},
				{z},
				{1.0}
			};
		}

		aeroforge::math::Matrix makeDirection(
			double x,
			double y,
			double z)
		{
			return aeroforge::math::Matrix{
				{x},
				{y},
				{z},
				{0.0}
			};
		}
	}

	// ============================================================
	// Translation
	// ============================================================

	TEST(TranslationTests, ProducesCorrectHomogeneousMatrix)
	{
		aeroforge::math::Matrix actual =
			translation(3.0, -2.0, 5.0);

		aeroforge::math::Matrix expected{
			{1.0, 0.0, 0.0, 3.0},
			{0.0, 1.0, 0.0, -2.0},
			{0.0, 0.0, 1.0, 5.0},
			{0.0, 0.0, 0.0, 1.0}
		};

		expectMatrixNear(actual, expected);
	}

	TEST(TranslationTests, ZeroTranslationProducesIdentity)
	{
		aeroforge::math::Matrix actual =
			translation(0.0, 0.0, 0.0);

		aeroforge::math::Matrix expected =
			aeroforge::math::Matrix::identity(4);

		expectMatrixNear(actual, expected);
	}

	TEST(TranslationTests, TranslatesHomogeneousPoint)
	{
		aeroforge::math::Matrix transform =
			translation(10.0, -5.0, 2.0);

		aeroforge::math::Matrix point =
			makePoint(1.0, 2.0, 3.0);

		aeroforge::math::Matrix result =
			transform * point;

		expectHomogeneousPointNear(
			result,
			11.0,
			-3.0,
			5.0);
	}

	TEST(TranslationTests, DoesNotTranslateDirectionVector)
	{
		aeroforge::math::Matrix transform =
			translation(10.0, -5.0, 2.0);

		aeroforge::math::Matrix direction =
			makeDirection(1.0, 2.0, 3.0);

		aeroforge::math::Matrix result =
			transform * direction;

		EXPECT_NEAR(result(0, 0), 1.0, EPSILON);
		EXPECT_NEAR(result(1, 0), 2.0, EPSILON);
		EXPECT_NEAR(result(2, 0), 3.0, EPSILON);
		EXPECT_NEAR(result(3, 0), 0.0, EPSILON);
	}

	// ============================================================
	// Rotation X
	// ============================================================

	TEST(RotationXTests, ZeroAngleProducesIdentity)
	{
		aeroforge::math::Matrix actual =
			rotationX(0.0);

		expectMatrixNear(
			actual,
			aeroforge::math::Matrix::identity(4));
	}

	TEST(RotationXTests, NinetyDegreesProducesCorrectMatrix)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix actual =
			rotationX(angle);

		aeroforge::math::Matrix expected{
			{1.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, -1.0, 0.0},
			{0.0, 1.0, 0.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		};

		expectMatrixNear(
			actual,
			expected,
			1e-12);
	}

	TEST(RotationXTests, PositiveNinetyRotatesPositiveYToPositiveZ)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix vector =
			makeDirection(0.0, 1.0, 0.0);

		aeroforge::math::Matrix result =
			rotationX(angle) * vector;

		EXPECT_NEAR(result(0, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(1, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(2, 0), 1.0, EPSILON);
	}

	TEST(RotationXTests, PositiveNinetyRotatesPositiveZToNegativeY)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix vector =
			makeDirection(0.0, 0.0, 1.0);

		aeroforge::math::Matrix result =
			rotationX(angle) * vector;

		EXPECT_NEAR(result(0, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(1, 0), -1.0, EPSILON);
		EXPECT_NEAR(result(2, 0), 0.0, EPSILON);
	}

	// ============================================================
	// Rotation Y
	// ============================================================

	TEST(RotationYTests, ZeroAngleProducesIdentity)
	{
		aeroforge::math::Matrix actual =
			rotationY(0.0);

		expectMatrixNear(
			actual,
			aeroforge::math::Matrix::identity(4));
	}

	TEST(RotationYTests, NinetyDegreesProducesCorrectMatrix)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix actual =
			rotationY(angle);

		aeroforge::math::Matrix expected{
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 1.0, 0.0, 0.0},
			{-1.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		};

		expectMatrixNear(
			actual,
			expected,
			1e-12);
	}

	TEST(RotationYTests, PositiveNinetyRotatesPositiveZToPositiveX)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix vector =
			makeDirection(0.0, 0.0, 1.0);

		aeroforge::math::Matrix result =
			rotationY(angle) * vector;

		EXPECT_NEAR(result(0, 0), 1.0, EPSILON);
		EXPECT_NEAR(result(1, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(2, 0), 0.0, EPSILON);
	}

	TEST(RotationYTests, PositiveNinetyRotatesPositiveXToNegativeZ)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix vector =
			makeDirection(1.0, 0.0, 0.0);

		aeroforge::math::Matrix result =
			rotationY(angle) * vector;

		EXPECT_NEAR(result(0, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(1, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(2, 0), -1.0, EPSILON);
	}

	// ============================================================
	// Rotation Z
	// ============================================================

	TEST(RotationZTests, ZeroAngleProducesIdentity)
	{
		aeroforge::math::Matrix actual =
			rotationZ(0.0);

		expectMatrixNear(
			actual,
			aeroforge::math::Matrix::identity(4));
	}

	TEST(RotationZTests, NinetyDegreesProducesCorrectMatrix)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix actual =
			rotationZ(angle);

		aeroforge::math::Matrix expected{
			{0.0, -1.0, 0.0, 0.0},
			{1.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		};

		expectMatrixNear(
			actual,
			expected,
			1e-12);
	}

	TEST(RotationZTests, PositiveNinetyRotatesPositiveXToPositiveY)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix vector =
			makeDirection(1.0, 0.0, 0.0);

		aeroforge::math::Matrix result =
			rotationZ(angle) * vector;

		EXPECT_NEAR(result(0, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(1, 0), 1.0, EPSILON);
		EXPECT_NEAR(result(2, 0), 0.0, EPSILON);
	}

	TEST(RotationZTests, PositiveNinetyRotatesPositiveYToNegativeX)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix vector =
			makeDirection(0.0, 1.0, 0.0);

		aeroforge::math::Matrix result =
			rotationZ(angle) * vector;

		EXPECT_NEAR(result(0, 0), -1.0, EPSILON);
		EXPECT_NEAR(result(1, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(2, 0), 0.0, EPSILON);
	}

	// ============================================================
	// Negative rotations
	// ============================================================

	TEST(RotationTests, NegativeNinetyRotationXReversesDirection)
	{
		const double angle =
			-std::numbers::pi / 2.0;

		aeroforge::math::Matrix vector =
			makeDirection(0.0, 1.0, 0.0);

		aeroforge::math::Matrix result =
			rotationX(angle) * vector;

		EXPECT_NEAR(result(0, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(1, 0), 0.0, EPSILON);
		EXPECT_NEAR(result(2, 0), -1.0, EPSILON);
	}

	TEST(RotationTests, FullRotationProducesIdentity)
	{
		const double angle =
			2.0 * std::numbers::pi;

		expectMatrixNear(
			rotationX(angle),
			aeroforge::math::Matrix::identity(4),
			1e-12);

		expectMatrixNear(
			rotationY(angle),
			aeroforge::math::Matrix::identity(4),
			1e-12);

		expectMatrixNear(
			rotationZ(angle),
			aeroforge::math::Matrix::identity(4),
			1e-12);
	}

	// ============================================================
	// Arbitrary-axis rotation
	// ============================================================

	TEST(RotationAxisTests, ZeroAngleProducesIdentity)
	{
		aeroforge::math::Vector3 axis(
			1.0,
			2.0,
			3.0);

		aeroforge::math::Matrix actual =
			rotationAxis(axis, 0.0);

		expectMatrixNear(
			actual,
			aeroforge::math::Matrix::identity(4),
			1e-12);
	}

	TEST(RotationAxisTests, UnitXMatchesRotationX)
	{
		const double angle = 0.735;

		aeroforge::math::Matrix axisRotation =
			rotationAxis(
				aeroforge::math::Vector3::unitX(),
				angle);

		aeroforge::math::Matrix xRotation =
			rotationX(angle);

		expectMatrixNear(
			axisRotation,
			xRotation,
			1e-12);
	}

	TEST(RotationAxisTests, UnitYMatchesRotationY)
	{
		const double angle = -1.2;

		aeroforge::math::Matrix axisRotation =
			rotationAxis(
				aeroforge::math::Vector3::unitY(),
				angle);

		aeroforge::math::Matrix yRotation =
			rotationY(angle);

		expectMatrixNear(
			axisRotation,
			yRotation,
			1e-12);
	}

	TEST(RotationAxisTests, UnitZMatchesRotationZ)
	{
		const double angle = 2.1;

		aeroforge::math::Matrix axisRotation =
			rotationAxis(
				aeroforge::math::Vector3::unitZ(),
				angle);

		aeroforge::math::Matrix zRotation =
			rotationZ(angle);

		expectMatrixNear(
			axisRotation,
			zRotation,
			1e-12);
	}

	TEST(RotationAxisTests, NonNormalizedAxisProducesSameRotation)
	{
		const double angle = 0.65;

		aeroforge::math::Vector3 unitAxis(
			1.0,
			0.0,
			0.0);

		aeroforge::math::Vector3 scaledAxis(
			10.0,
			0.0,
			0.0);

		aeroforge::math::Matrix unitResult =
			rotationAxis(
				unitAxis,
				angle);

		aeroforge::math::Matrix scaledResult =
			rotationAxis(
				scaledAxis,
				angle);

		expectMatrixNear(
			unitResult,
			scaledResult,
			1e-12);
	}

	TEST(RotationAxisTests, ZeroAxisThrows)
	{
		aeroforge::math::Vector3 zeroAxis(
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			rotationAxis(
				zeroAxis,
				1.0),
			std::domain_error);
	}

	TEST(RotationAxisTests, AxisItselfIsUnchangedByRotation)
	{
		aeroforge::math::Vector3 axis(
			1.0,
			2.0,
			3.0);

		aeroforge::math::Vector3 normalized =
			axis.normalize();

		const double angle = 1.1;

		aeroforge::math::Matrix rotation =
			rotationAxis(
				axis,
				angle);

		aeroforge::math::Matrix axisVector =
			makeDirection(
				normalized.x,
				normalized.y,
				normalized.z);

		aeroforge::math::Matrix result =
			rotation * axisVector;

		EXPECT_NEAR(
			result(0, 0),
			normalized.x,
			1e-12);

		EXPECT_NEAR(
			result(1, 0),
			normalized.y,
			1e-12);

		EXPECT_NEAR(
			result(2, 0),
			normalized.z,
			1e-12);
	}

	// ============================================================
	// Rotation matrix properties
	// ============================================================

	TEST(RotationPropertyTests, RotationXIsOrthogonal)
	{
		aeroforge::math::Matrix rotation =
			rotationX(0.73);

		aeroforge::math::Matrix rotation3 =
			rotation.subMatrix(
				0,
				0,
				3,
				3);

		aeroforge::math::Matrix product =
			rotation3.transpose() * rotation3;

		expectMatrixNear(
			product,
			aeroforge::math::Matrix::identity(3),
			1e-12);
	}

	TEST(RotationPropertyTests, RotationYIsOrthogonal)
	{
		aeroforge::math::Matrix rotation =
			rotationY(-1.31);

		aeroforge::math::Matrix rotation3 =
			rotation.subMatrix(
				0,
				0,
				3,
				3);

		aeroforge::math::Matrix product =
			rotation3.transpose() * rotation3;

		expectMatrixNear(
			product,
			aeroforge::math::Matrix::identity(3),
			1e-12);
	}

	TEST(RotationPropertyTests, RotationZIsOrthogonal)
	{
		aeroforge::math::Matrix rotation =
			rotationZ(2.47);

		aeroforge::math::Matrix rotation3 =
			rotation.subMatrix(
				0,
				0,
				3,
				3);

		aeroforge::math::Matrix product =
			rotation3.transpose() * rotation3;

		expectMatrixNear(
			product,
			aeroforge::math::Matrix::identity(3),
			1e-12);
	}

	TEST(RotationPropertyTests, ArbitraryAxisRotationIsOrthogonal)
	{
		aeroforge::math::Matrix rotation =
			rotationAxis(
				aeroforge::math::Vector3(
					1.0,
					2.0,
					-3.0),
				0.84);

		aeroforge::math::Matrix rotation3 =
			rotation.subMatrix(
				0,
				0,
				3,
				3);

		aeroforge::math::Matrix product =
			rotation3.transpose() * rotation3;

		expectMatrixNear(
			product,
			aeroforge::math::Matrix::identity(3),
			1e-12);
	}

	TEST(RotationPropertyTests, RotationDeterminantIsOne)
	{
		aeroforge::math::Matrix x =
			rotationX(0.4)
			.subMatrix(0, 0, 3, 3);

		aeroforge::math::Matrix y =
			rotationY(-0.9)
			.subMatrix(0, 0, 3, 3);

		aeroforge::math::Matrix z =
			rotationZ(1.7)
			.subMatrix(0, 0, 3, 3);

		EXPECT_NEAR(
			x.determinant(),
			1.0,
			1e-12);

		EXPECT_NEAR(
			y.determinant(),
			1.0,
			1e-12);

		EXPECT_NEAR(
			z.determinant(),
			1.0,
			1e-12);
	}

	TEST(RotationPropertyTests, ArbitraryAxisRotationDeterminantIsOne)
	{
		aeroforge::math::Matrix rotation =
			rotationAxis(
				aeroforge::math::Vector3(
					1.0,
					4.0,
					2.0),
				1.45);

		aeroforge::math::Matrix rotation3 =
			rotation.subMatrix(
				0,
				0,
				3,
				3);

		EXPECT_NEAR(
			rotation3.determinant(),
			1.0,
			1e-12);
	}

	TEST(RotationPropertyTests, InverseEqualsTranspose)
	{
		aeroforge::math::Matrix rotation =
			rotationAxis(
				aeroforge::math::Vector3(
					2.0,
					-1.0,
					4.0),
				0.92);

		aeroforge::math::Matrix rotation3 =
			rotation.subMatrix(
				0,
				0,
				3,
				3);

		expectMatrixNear(
			rotation3.inverse(),
			rotation3.transpose(),
			1e-12);
	}

	TEST(RotationPropertyTests, RotationPreservesVectorMagnitude)
	{
		aeroforge::math::Vector3 original(
			3.0,
			-4.0,
			7.0);

		aeroforge::math::Matrix rotation =
			rotationAxis(
				aeroforge::math::Vector3(
					1.0,
					2.0,
					3.0),
				1.23);

		aeroforge::math::Matrix rotation3 =
			rotation.subMatrix(
				0,
				0,
				3,
				3);

		aeroforge::math::Vector3 rotated =
			rotation3 * original;

		EXPECT_NEAR(
			rotated.magnitude(),
			original.magnitude(),
			1e-12);
	}

	// ============================================================
	// Homogeneous matrix structure
	// ============================================================

	TEST(TransformStructureTests, RotationMatricesHaveNoTranslation)
	{
		aeroforge::math::Matrix rotations[] = {
			rotationX(0.5),
			rotationY(0.5),
			rotationZ(0.5),
			rotationAxis(
				aeroforge::math::Vector3(
					1.0,
					2.0,
					3.0),
				0.5)
		};

		for (const auto& matrix : rotations)
		{
			ASSERT_EQ(matrix.rows(), 4);
			ASSERT_EQ(matrix.columns(), 4);

			EXPECT_NEAR(
				matrix(0, 3),
				0.0,
				EPSILON);

			EXPECT_NEAR(
				matrix(1, 3),
				0.0,
				EPSILON);

			EXPECT_NEAR(
				matrix(2, 3),
				0.0,
				EPSILON);

			EXPECT_NEAR(
				matrix(3, 0),
				0.0,
				EPSILON);

			EXPECT_NEAR(
				matrix(3, 1),
				0.0,
				EPSILON);

			EXPECT_NEAR(
				matrix(3, 2),
				0.0,
				EPSILON);

			EXPECT_NEAR(
				matrix(3, 3),
				1.0,
				EPSILON);
		}
	}

	// ============================================================
	// Transform composition
	// ============================================================

	TEST(TransformCompositionTests, TranslationCompositionAddsTranslations)
	{
		aeroforge::math::Matrix first =
			translation(
				1.0,
				2.0,
				3.0);

		aeroforge::math::Matrix second =
			translation(
				4.0,
				-1.0,
				2.0);

		aeroforge::math::Matrix combined =
			second * first;

		aeroforge::math::Matrix expected =
			translation(
				5.0,
				1.0,
				5.0);

		expectMatrixNear(
			combined,
			expected,
			1e-12);
	}

	TEST(TransformCompositionTests, RotationFollowedByInverseReturnsOriginal)
	{
		const double angle = 0.87;

		aeroforge::math::Matrix forward =
			rotationZ(angle);

		aeroforge::math::Matrix backward =
			rotationZ(-angle);

		aeroforge::math::Matrix result =
			backward * forward;

		expectMatrixNear(
			result,
			aeroforge::math::Matrix::identity(4),
			1e-12);
	}

	TEST(TransformCompositionTests, MatrixOrderAffectsRotationAndTranslation)
	{
		const double angle =
			std::numbers::pi / 2.0;

		aeroforge::math::Matrix point =
			makePoint(
				1.0,
				0.0,
				0.0);

		aeroforge::math::Matrix rotateThenTranslate =
			translation(
				1.0,
				0.0,
				0.0)
			* rotationZ(angle);

		aeroforge::math::Matrix result =
			rotateThenTranslate * point;

		// Point:
		// (1, 0, 0)
		//
		// Rotate +90 degrees about Z:
		// (0, 1, 0)
		//
		// Translate +1 along X:
		// (1, 1, 0)

		expectHomogeneousPointNear(
			result,
			1.0,
			1.0,
			0.0);
	}
}