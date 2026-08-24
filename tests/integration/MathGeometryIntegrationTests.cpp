#include <gtest/gtest.h>

#include <geometry/FrameTransform.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>

namespace aeroforge::integration::tests
{
	namespace
	{
		constexpr double EPSILON = 1e-9;
		constexpr double RANDOM_EPSILON = 1e-8;
		constexpr double PI =
			std::numbers::pi_v<double>;

		void expectScalarApprox(
			double actual,
			double expected,
			double epsilon = EPSILON)
		{
			const double scale =
				std::max({
					1.0,
					std::abs(actual),
					std::abs(expected)
					});

			EXPECT_LE(
				std::abs(actual - expected),
				epsilon * scale);
		}

		void expectVectorApprox(
			const math::Vector3& actual,
			const math::Vector3& expected,
			double epsilon = EPSILON)
		{
			expectScalarApprox(
				actual.x,
				expected.x,
				epsilon);

			expectScalarApprox(
				actual.y,
				expected.y,
				epsilon);

			expectScalarApprox(
				actual.z,
				expected.z,
				epsilon);
		}

		void expectMatrixApprox(
			const math::Matrix& actual,
			const math::Matrix& expected,
			double epsilon = EPSILON)
		{
			ASSERT_EQ(
				actual.rows(),
				expected.rows());

			ASSERT_EQ(
				actual.columns(),
				expected.columns());

			for (std::size_t row = 0;
				row < actual.rows();
				++row)
			{
				for (std::size_t column = 0;
					column < actual.columns();
					++column)
				{
					expectScalarApprox(
						actual(row, column),
						expected(row, column),
						epsilon);
				}
			}
		}

		void expectSameRotation(
			const math::Quaternion& actual,
			const math::Quaternion& expected,
			double epsilon = EPSILON)
		{
			EXPECT_TRUE(
				actual.isSameRotation(
					expected,
					epsilon));
		}

		math::Vector3 normalized(
			const math::Vector3& vector)
		{
			return vector /
				vector.magnitude();
		}
	}

	// ============================================================
	// Quaternion / Matrix / Vector consistency
	// ============================================================

	TEST(
		MathGeometryQuaternionMatrixTests,
		QuaternionRotationMatchesRotationMatrix)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					-2.0,
					3.0),
				1.37);

		const math::Vector3 vector(
			4.0,
			-7.0,
			2.5);

		const math::Vector3 quaternionResult =
			q.rotate(vector);

		const math::Vector3 matrixResult =
			q.toRotationMatrix() *
			vector;

		expectVectorApprox(
			quaternionResult,
			matrixResult);
	}

	TEST(
		MathGeometryQuaternionMatrixTests,
		BodyToNavigationMatchesRotationMatrix)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					3.0,
					-1.0),
				0.91);

		const math::Vector3 vectorBody(
			5.0,
			-4.0,
			8.0);

		const math::Vector3 quaternionResult =
			geometry::bodyToNavigation(
				vectorBody,
				qNB);

		const math::Vector3 matrixResult =
			geometry::bodyToNavigationMatrix(
				qNB) *
			vectorBody;

		expectVectorApprox(
			quaternionResult,
			matrixResult);
	}

	TEST(
		MathGeometryQuaternionMatrixTests,
		NavigationToBodyMatchesRotationMatrix)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-4.0,
					1.0,
					2.0),
				1.12);

		const math::Vector3 vectorNavigation(
			-6.0,
			2.0,
			9.0);

		const math::Vector3 quaternionResult =
			geometry::navigationToBody(
				vectorNavigation,
				qNB);

		const math::Vector3 matrixResult =
			geometry::navigationToBodyMatrix(
				qNB) *
			vectorNavigation;

		expectVectorApprox(
			quaternionResult,
			matrixResult);
	}

	TEST(
		MathGeometryQuaternionMatrixTests,
		ReverseRotationMatrixIsTranspose)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					3.0,
					-2.0,
					5.0),
				1.71);

		const math::Matrix RNB =
			geometry::bodyToNavigationMatrix(
				qNB);

		const math::Matrix RBN =
			geometry::navigationToBodyMatrix(
				qNB);

		expectMatrixApprox(
			RBN,
			RNB.transpose());
	}

	TEST(
		MathGeometryQuaternionMatrixTests,
		ForwardAndReverseMatricesAreMutualInverses)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					4.0,
					2.0),
				0.72);

		const math::Matrix RNB =
			geometry::bodyToNavigationMatrix(
				qNB);

		const math::Matrix RBN =
			geometry::navigationToBodyMatrix(
				qNB);

		expectMatrixApprox(
			RNB * RBN,
			math::Matrix::identity(3));

		expectMatrixApprox(
			RBN * RNB,
			math::Matrix::identity(3));
	}

	// ============================================================
	// Quaternion <-> Matrix round trips
	// ============================================================

	TEST(
		MathGeometryRoundTripTests,
		IdentityQuaternionMatrixRoundTrip)
	{
		const math::Quaternion original =
			math::Quaternion::identity();

		const math::Quaternion recovered =
			math::Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original);
	}

	TEST(
		MathGeometryRoundTripTests,
		XRotationQuaternionMatrixRoundTrip)
	{
		const math::Quaternion original =
			math::Quaternion::fromRotationX(
				PI / 2.0);

		const math::Quaternion recovered =
			math::Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original);
	}

	TEST(
		MathGeometryRoundTripTests,
		YRotationQuaternionMatrixRoundTrip)
	{
		const math::Quaternion original =
			math::Quaternion::fromRotationY(
				PI / 2.0);

		const math::Quaternion recovered =
			math::Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original);
	}

	TEST(
		MathGeometryRoundTripTests,
		ZRotationQuaternionMatrixRoundTrip)
	{
		const math::Quaternion original =
			math::Quaternion::fromRotationZ(
				PI / 2.0);

		const math::Quaternion recovered =
			math::Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original);
	}

	TEST(
		MathGeometryRoundTripTests,
		OneHundredEightyDegreeRotationRoundTrip)
	{
		const math::Quaternion original =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					3.0),
				PI);

		const math::Quaternion recovered =
			math::Quaternion::fromRotationMatrix(
				original.toRotationMatrix());

		expectSameRotation(
			recovered,
			original);
	}

	TEST(
		MathGeometryRoundTripTests,
		ArbitraryRotationRoundTrip)
	{
		const math::Quaternion original =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-3.0,
					5.0,
					2.0),
				1.834);

		const math::Matrix matrix =
			original.toRotationMatrix();

		const math::Quaternion recovered =
			math::Quaternion::fromRotationMatrix(
				matrix);

		expectSameRotation(
			recovered,
			original);

		expectMatrixApprox(
			recovered.toRotationMatrix(),
			matrix);
	}

	TEST(
		MathGeometryRoundTripTests,
		NegativeQuaternionProducesSameRoundTrip)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					-1.0,
					4.0),
				1.31);

		const math::Quaternion negative =
			q * -1.0;

		expectMatrixApprox(
			q.toRotationMatrix(),
			negative.toRotationMatrix());

		const math::Quaternion recovered =
			math::Quaternion::fromRotationMatrix(
				negative.toRotationMatrix());

		expectSameRotation(
			recovered,
			q);
	}

	// ============================================================
	// Rotation composition
	// ============================================================

	TEST(
		MathGeometryCompositionTests,
		QuaternionCompositionMatchesMatrixComposition)
	{
		const math::Quaternion q1 =
			math::Quaternion::fromRotationX(
				0.4);

		const math::Quaternion q2 =
			math::Quaternion::fromRotationY(
				-0.7);

		const math::Quaternion combined =
			q2 * q1;

		const math::Matrix matrixCombined =
			q2.toRotationMatrix() *
			q1.toRotationMatrix();

		expectMatrixApprox(
			combined.toRotationMatrix(),
			matrixCombined);
	}

	TEST(
		MathGeometryCompositionTests,
		QuaternionCompositionMatchesSequentialVectorRotations)
	{
		const math::Quaternion q1 =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					0.5),
				0.6);

		const math::Quaternion q2 =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-2.0,
					1.0,
					3.0),
				1.1);

		const math::Vector3 vector(
			3.0,
			-5.0,
			2.0);

		const math::Vector3 sequential =
			q2.rotate(
				q1.rotate(vector));

		const math::Vector3 combined =
			(q2 * q1).rotate(
				vector);

		expectVectorApprox(
			combined,
			sequential);
	}

	TEST(
		MathGeometryCompositionTests,
		RollPitchYawCompositionMatchesMatrixOrder)
	{
		const math::Quaternion roll =
			math::Quaternion::fromRotationX(
				0.31);

		const math::Quaternion pitch =
			math::Quaternion::fromRotationY(
				-0.62);

		const math::Quaternion yaw =
			math::Quaternion::fromRotationZ(
				1.27);

		/*
		 * Column-vector convention:
		 *
		 * q = yaw * pitch * roll
		 *
		 * applies:
		 *
		 * roll -> pitch -> yaw
		 */
		const math::Quaternion qCombined =
			yaw * pitch * roll;

		const math::Matrix RCombined =
			yaw.toRotationMatrix() *
			pitch.toRotationMatrix() *
			roll.toRotationMatrix();

		expectMatrixApprox(
			qCombined.toRotationMatrix(),
			RCombined);
	}

	// ============================================================
	// NED / FRD convention lock-in
	// ============================================================

	TEST(
		MathGeometryConventionTests,
		IdentityMapsForwardToNorth)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		expectVectorApprox(
			geometry::bodyToNavigation(
				math::Vector3::unitX(),
				qNB),
			math::Vector3::unitX());
	}

	TEST(
		MathGeometryConventionTests,
		IdentityMapsRightToEast)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		expectVectorApprox(
			geometry::bodyToNavigation(
				math::Vector3::unitY(),
				qNB),
			math::Vector3::unitY());
	}

	TEST(
		MathGeometryConventionTests,
		IdentityMapsBodyDownToNavigationDown)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		expectVectorApprox(
			geometry::bodyToNavigation(
				math::Vector3::unitZ(),
				qNB),
			math::Vector3::unitZ());
	}

	TEST(
		MathGeometryConventionTests,
		PositiveNinetyDegreeYawMapsForwardToEast)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationZ(
				PI / 2.0);

		expectVectorApprox(
			geometry::bodyToNavigation(
				math::Vector3::unitX(),
				qNB),
			math::Vector3::unitY());
	}

	TEST(
		MathGeometryConventionTests,
		PositiveNinetyDegreePitchMapsForwardUp)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationY(
				PI / 2.0);

		/*
		 * Navigation uses NED:
		 *
		 * +Z = down
		 * -Z = up
		 */
		expectVectorApprox(
			geometry::bodyToNavigation(
				math::Vector3::unitX(),
				qNB),
			math::Vector3(
				0.0,
				0.0,
				-1.0));
	}

	TEST(
		MathGeometryConventionTests,
		PositiveNinetyDegreeRollMapsRightToDown)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationX(
				PI / 2.0);

		expectVectorApprox(
			geometry::bodyToNavigation(
				math::Vector3::unitY(),
				qNB),
			math::Vector3::unitZ());
	}

	// ============================================================
	// Rotation invariants
	// ============================================================

	TEST(
		MathGeometryInvariantTests,
		QuaternionRotationPreservesMagnitude)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					-4.0,
					1.0),
				1.27);

		const math::Vector3 vector(
			5.0,
			-9.0,
			3.0);

		const math::Vector3 rotated =
			q.rotate(vector);

		expectScalarApprox(
			rotated.magnitude(),
			vector.magnitude());
	}

	TEST(
		MathGeometryInvariantTests,
		MatrixRotationPreservesMagnitude)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					5.0,
					-2.0),
				0.87);

		const math::Vector3 vector(
			7.0,
			3.0,
			-6.0);

		const math::Vector3 rotated =
			q.toRotationMatrix() *
			vector;

		expectScalarApprox(
			rotated.magnitude(),
			vector.magnitude());
	}

	TEST(
		MathGeometryInvariantTests,
		RotationPreservesDotProduct)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					3.0,
					4.0),
				1.51);

		const math::Vector3 a(
			1.0,
			-4.0,
			2.0);

		const math::Vector3 b(
			5.0,
			3.0,
			-7.0);

		const math::Vector3 rotatedA =
			q.rotate(a);

		const math::Vector3 rotatedB =
			q.rotate(b);

		expectScalarApprox(
			rotatedA.dot(rotatedB),
			a.dot(b));
	}

	TEST(
		MathGeometryInvariantTests,
		RotationPreservesCrossProductRelationship)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-3.0,
					2.0,
					1.0),
				0.94);

		const math::Vector3 a(
			4.0,
			1.0,
			-2.0);

		const math::Vector3 b(
			-1.0,
			5.0,
			3.0);

		const math::Vector3 left =
			q.rotate(
				a.cross(b));

		const math::Vector3 right =
			q.rotate(a)
			.cross(
				q.rotate(b));

		expectVectorApprox(
			left,
			right);
	}

	// ============================================================
	// Rotation matrix properties
	// ============================================================

	TEST(
		MathGeometryMatrixPropertiesTests,
		RotationMatrixTransposeTimesMatrixIsIdentity)
	{
		const math::Matrix rotation =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					3.0,
					-5.0),
				1.42)
			.toRotationMatrix();

		expectMatrixApprox(
			rotation.transpose() *
			rotation,
			math::Matrix::identity(3));
	}

	TEST(
		MathGeometryMatrixPropertiesTests,
		RotationMatrixTimesTransposeIsIdentity)
	{
		const math::Matrix rotation =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					4.0,
					-2.0,
					3.0),
				0.73)
			.toRotationMatrix();

		expectMatrixApprox(
			rotation *
			rotation.transpose(),
			math::Matrix::identity(3));
	}

	TEST(
		MathGeometryMatrixPropertiesTests,
		RotationMatrixDeterminantIsPositiveOne)
	{
		const math::Matrix rotation =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					5.0,
					1.0),
				2.14)
			.toRotationMatrix();

		expectScalarApprox(
			rotation.determinant(),
			1.0);
	}

	TEST(
		MathGeometryMatrixPropertiesTests,
		RotationMatrixInverseEqualsTranspose)
	{
		const math::Matrix rotation =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					-3.0,
					2.0),
				1.21)
			.toRotationMatrix();

		expectMatrixApprox(
			rotation.inverse(),
			rotation.transpose());
	}

	// ============================================================
	// Quaternion algebra properties
	// ============================================================

	TEST(
		MathGeometryQuaternionPropertiesTests,
		QuaternionTimesInverseIsIdentity)
	{
		const math::Quaternion q(
			1.0,
			2.0,
			-3.0,
			4.0);

		const math::Quaternion result =
			q * q.inverse();

		expectSameRotation(
			result,
			math::Quaternion::identity());
	}

	TEST(
		MathGeometryQuaternionPropertiesTests,
		InverseTimesQuaternionIsIdentity)
	{
		const math::Quaternion q(
			2.0,
			-1.0,
			5.0,
			3.0);

		const math::Quaternion result =
			q.inverse() * q;

		expectSameRotation(
			result,
			math::Quaternion::identity());
	}

	TEST(
		MathGeometryQuaternionPropertiesTests,
		UnitQuaternionInverseEqualsConjugate)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					-2.0,
					4.0),
				1.3);

		EXPECT_TRUE(
			q.inverse().isApprox(
				q.conjugate(),
				EPSILON));
	}

	TEST(
		MathGeometryQuaternionPropertiesTests,
		ProductNormEqualsProductOfNorms)
	{
		const math::Quaternion a(
			1.0,
			2.0,
			3.0,
			4.0);

		const math::Quaternion b(
			-2.0,
			5.0,
			1.0,
			3.0);

		expectScalarApprox(
			(a * b).norm(),
			a.norm() * b.norm());
	}

	TEST(
		MathGeometryQuaternionPropertiesTests,
		PositiveAndNegativeQuaternionRotateIdentically)
	{
		const math::Quaternion q =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					3.0),
				1.4);

		const math::Quaternion negative =
			q * -1.0;

		const math::Vector3 vector(
			5.0,
			-3.0,
			7.0);

		expectVectorApprox(
			q.rotate(vector),
			negative.rotate(vector));

		expectMatrixApprox(
			q.toRotationMatrix(),
			negative.toRotationMatrix());

		EXPECT_TRUE(
			q.isSameRotation(
				negative,
				EPSILON));
	}

	// ============================================================
	// Frame round trips
	// ============================================================

	TEST(
		MathGeometryFrameRoundTripTests,
		BodyNavigationBodyRestoresVector)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-1.0,
					4.0,
					2.0),
				1.53);

		const math::Vector3 original(
			7.0,
			-2.0,
			5.0);

		const math::Vector3 navigation =
			geometry::bodyToNavigation(
				original,
				qNB);

		const math::Vector3 restored =
			geometry::navigationToBody(
				navigation,
				qNB);

		expectVectorApprox(
			restored,
			original);
	}

	TEST(
		MathGeometryFrameRoundTripTests,
		NavigationBodyNavigationRestoresVector)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					3.0,
					1.0,
					-2.0),
				0.81);

		const math::Vector3 original(
			-8.0,
			4.0,
			6.0);

		const math::Vector3 body =
			geometry::navigationToBody(
				original,
				qNB);

		const math::Vector3 restored =
			geometry::bodyToNavigation(
				body,
				qNB);

		expectVectorApprox(
			restored,
			original);
	}

	// ============================================================
	// Numerical edge cases
	// ============================================================

	TEST(
		MathGeometryEdgeCaseTests,
		NearZeroAndNearPiRotationsRoundTrip)
	{
		const std::array<double, 5> angles{
			1e-12,
			PI - 1e-12,
			PI,
			PI + 1e-12,
			2.0 * PI - 1e-12
		};

		const math::Vector3 axis(
			1.0,
			-2.0,
			3.0);

		for (double angle : angles)
		{
			const math::Quaternion original =
				math::Quaternion::fromAxisAngle(
					axis,
					angle);

			ASSERT_TRUE(
				original.isFinite());

			const math::Matrix matrix =
				original.toRotationMatrix();

			ASSERT_TRUE(
				matrix.isFinite());

			const math::Quaternion recovered =
				math::Quaternion::fromRotationMatrix(
					matrix);

			EXPECT_TRUE(
				original.isSameRotation(
					recovered,
					1e-8));

			expectMatrixApprox(
				recovered.toRotationMatrix(),
				matrix,
				1e-8);
		}
	}

	TEST(
		MathGeometryEdgeCaseTests,
		LargeVectorRoundTrip)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					3.0),
				1.0);

		const math::Vector3 original(
			1e100,
			-2e100,
			3e100);

		const math::Vector3 transformed =
			geometry::bodyToNavigation(
				original,
				qNB);

		const math::Vector3 restored =
			geometry::navigationToBody(
				transformed,
				qNB);

		expectVectorApprox(
			restored,
			original,
			1e-8);
	}

	TEST(
		MathGeometryEdgeCaseTests,
		SmallNonzeroVectorRoundTrip)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-2.0,
					1.0,
					4.0),
				1.3);

		const math::Vector3 original(
			1e-100,
			-2e-100,
			3e-100);

		const math::Vector3 transformed =
			geometry::bodyToNavigation(
				original,
				qNB);

		const math::Vector3 restored =
			geometry::navigationToBody(
				transformed,
				qNB);

		expectVectorApprox(
			restored,
			original,
			1e-8);
	}

	TEST(
		MathGeometryEdgeCaseTests,
		ScaledQuaternionProducesSameFrameTransform)
	{
		const math::Quaternion unit =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					-3.0),
				1.19);

		const math::Quaternion scaled =
			unit * 100.0;

		const math::Vector3 vector(
			4.0,
			-8.0,
			2.0);

		expectVectorApprox(
			geometry::bodyToNavigation(
				vector,
				scaled),
			geometry::bodyToNavigation(
				vector,
				unit));

		expectMatrixApprox(
			geometry::bodyToNavigationMatrix(
				scaled),
			geometry::bodyToNavigationMatrix(
				unit));
	}

	TEST(
		MathGeometryEdgeCaseTests,
		NearlyParallelVectorsProduceCorrectRotation)
	{
		const math::Vector3 from(
			1.0,
			0.0,
			0.0);

		const math::Vector3 to(
			1.0,
			1e-4,
			0.0);

		const math::Quaternion q =
			math::Quaternion::fromTwoVectors(
				from,
				to);

		const math::Vector3 actual =
			normalized(
				q.rotate(from));

		const math::Vector3 expected =
			normalized(to);

		expectVectorApprox(
			actual,
			expected,
			1e-8);
	}

	TEST(
		MathGeometryEdgeCaseTests,
		NearlyOppositeVectorsProduceCorrectRotation)
	{
		const math::Vector3 from(
			1.0,
			0.0,
			0.0);

		const math::Vector3 to(
			-1.0,
			1e-4,
			0.0);

		const math::Quaternion q =
			math::Quaternion::fromTwoVectors(
				from,
				to);

		const math::Vector3 actual =
			normalized(
				q.rotate(from));

		const math::Vector3 expected =
			normalized(to);

		expectVectorApprox(
			actual,
			expected,
			1e-8);
	}

	// ============================================================
	// Deterministic randomized integration verification
	// ============================================================

	TEST(
		MathGeometryRandomizedTests,
		OneThousandRandomizedRotationCases)
	{
		std::mt19937_64 generator(
			0xA3E0F04EULL);

		std::uniform_real_distribution<double>
			componentDistribution(
				-100.0,
				100.0);

		std::uniform_real_distribution<double>
			angleDistribution(
				-PI,
				PI);

		for (int iteration = 0;
			iteration < 1000;
			++iteration)
		{
			math::Vector3 axis(
				componentDistribution(generator),
				componentDistribution(generator),
				componentDistribution(generator));

			if (axis.magnitude() < 1e-6)
			{
				axis =
					math::Vector3::unitX();
			}

			const double angle =
				angleDistribution(generator);

			const math::Quaternion qNB =
				math::Quaternion::fromAxisAngle(
					axis,
					angle);

			ASSERT_TRUE(
				qNB.isFinite());

			ASSERT_TRUE(
				qNB.isNormalized(
					RANDOM_EPSILON));

			const math::Matrix RNB =
				geometry::bodyToNavigationMatrix(
					qNB);

			const math::Matrix RBN =
				geometry::navigationToBodyMatrix(
					qNB);

			ASSERT_TRUE(
				RNB.isFinite());

			ASSERT_TRUE(
				RBN.isFinite());

			const math::Vector3 vectorBody(
				componentDistribution(generator),
				componentDistribution(generator),
				componentDistribution(generator));

			const math::Vector3 secondVectorBody(
				componentDistribution(generator),
				componentDistribution(generator),
				componentDistribution(generator));

			// ----------------------------------------------------
			// Quaternion and matrix transformations agree.
			// ----------------------------------------------------

			const math::Vector3 vectorNavigation =
				geometry::bodyToNavigation(
					vectorBody,
					qNB);

			const math::Vector3 matrixNavigation =
				RNB * vectorBody;

			expectVectorApprox(
				vectorNavigation,
				matrixNavigation,
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Body -> Navigation -> Body round trip.
			// ----------------------------------------------------

			const math::Vector3 restoredBody =
				geometry::navigationToBody(
					vectorNavigation,
					qNB);

			expectVectorApprox(
				restoredBody,
				vectorBody,
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Magnitude preservation.
			// ----------------------------------------------------

			expectScalarApprox(
				vectorNavigation.magnitude(),
				vectorBody.magnitude(),
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Rotation matrix orthogonality.
			// ----------------------------------------------------

			expectMatrixApprox(
				RNB.transpose() * RNB,
				math::Matrix::identity(3),
				RANDOM_EPSILON);

			expectMatrixApprox(
				RNB * RNB.transpose(),
				math::Matrix::identity(3),
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Forward / reverse relationship.
			// ----------------------------------------------------

			expectMatrixApprox(
				RBN,
				RNB.transpose(),
				RANDOM_EPSILON);

			expectMatrixApprox(
				RBN * RNB,
				math::Matrix::identity(3),
				RANDOM_EPSILON);

			expectMatrixApprox(
				RNB * RBN,
				math::Matrix::identity(3),
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Proper rotation determinant.
			// ----------------------------------------------------

			expectScalarApprox(
				RNB.determinant(),
				1.0,
				RANDOM_EPSILON);

			expectScalarApprox(
				RBN.determinant(),
				1.0,
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Matrix inverse must equal transpose.
			// ----------------------------------------------------

			expectMatrixApprox(
				RNB.inverse(),
				RNB.transpose(),
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Quaternion -> Matrix -> Quaternion.
			// ----------------------------------------------------

			const math::Quaternion recovered =
				math::Quaternion::fromRotationMatrix(
					RNB);

			EXPECT_TRUE(
				qNB.isSameRotation(
					recovered,
					RANDOM_EPSILON));

			// ----------------------------------------------------
			// Matrix -> Quaternion -> Matrix.
			// ----------------------------------------------------

			expectMatrixApprox(
				recovered.toRotationMatrix(),
				RNB,
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Dot-product preservation.
			// ----------------------------------------------------

			const math::Vector3 firstRotated =
				qNB.rotate(
					vectorBody);

			const math::Vector3 secondRotated =
				qNB.rotate(
					secondVectorBody);

			expectScalarApprox(
				firstRotated.dot(
					secondRotated),
				vectorBody.dot(
					secondVectorBody),
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Cross-product preservation.
			// ----------------------------------------------------

			const math::Vector3 rotatedCross =
				qNB.rotate(
					vectorBody.cross(
						secondVectorBody));

			const math::Vector3 crossOfRotated =
				firstRotated.cross(
					secondRotated);

			expectVectorApprox(
				rotatedCross,
				crossOfRotated,
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// q and -q represent identical orientations.
			// ----------------------------------------------------

			const math::Quaternion negative =
				qNB * -1.0;

			EXPECT_TRUE(
				qNB.isSameRotation(
					negative,
					RANDOM_EPSILON));

			expectVectorApprox(
				negative.rotate(
					vectorBody),
				vectorNavigation,
				RANDOM_EPSILON);

			expectMatrixApprox(
				negative.toRotationMatrix(),
				RNB,
				RANDOM_EPSILON);

			// ----------------------------------------------------
			// Numerical safety.
			// ----------------------------------------------------

			EXPECT_TRUE(
				vectorNavigation.isFinite());

			EXPECT_TRUE(
				restoredBody.isFinite());

			EXPECT_TRUE(
				recovered.isFinite());
		}
	}
}