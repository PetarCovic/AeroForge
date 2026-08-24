#include <gtest/gtest.h>

#include <geometry/FrameTransform.h>
#include <math/Vector3.h>
#include <math/Matrix.h>
#include <math/Quaternion.h>

#include <cmath>
#include <numbers>
#include <random>

namespace aeroforge::geometry::tests
{
	namespace
	{
		constexpr double EPSILON = 1e-9;
		constexpr double PI =
			std::numbers::pi_v<double>;

		void expectVectorNear(
			const math::Vector3& actual,
			const math::Vector3& expected,
			double epsilon = EPSILON)
		{
			EXPECT_NEAR(
				actual.x,
				expected.x,
				epsilon);

			EXPECT_NEAR(
				actual.y,
				expected.y,
				epsilon);

			EXPECT_NEAR(
				actual.z,
				expected.z,
				epsilon);
		}

		void expectMatrixNear(
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
					EXPECT_NEAR(
						actual(row, column),
						expected(row, column),
						epsilon);
				}
			}
		}
	}

	// ============================================================
	// Identity attitude
	// ============================================================

	TEST(FrameTransformIdentityTests, BodyXMapsToNavigationX)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		const math::Vector3 result =
			bodyToNavigation(
				math::Vector3::unitX(),
				qNB);

		expectVectorNear(
			result,
			math::Vector3::unitX());
	}

	TEST(FrameTransformIdentityTests, BodyYMapsToNavigationY)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		const math::Vector3 result =
			bodyToNavigation(
				math::Vector3::unitY(),
				qNB);

		expectVectorNear(
			result,
			math::Vector3::unitY());
	}

	TEST(FrameTransformIdentityTests, BodyZMapsToNavigationZ)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		const math::Vector3 result =
			bodyToNavigation(
				math::Vector3::unitZ(),
				qNB);

		expectVectorNear(
			result,
			math::Vector3::unitZ());
	}

	TEST(FrameTransformIdentityTests, NavigationXMapsToBodyX)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		expectVectorNear(
			navigationToBody(
				math::Vector3::unitX(),
				qNB),
			math::Vector3::unitX());
	}

	TEST(FrameTransformIdentityTests, NavigationYMapsToBodyY)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		expectVectorNear(
			navigationToBody(
				math::Vector3::unitY(),
				qNB),
			math::Vector3::unitY());
	}

	TEST(FrameTransformIdentityTests, NavigationZMapsToBodyZ)
	{
		const math::Quaternion qNB =
			math::Quaternion::identity();

		expectVectorNear(
			navigationToBody(
				math::Vector3::unitZ(),
				qNB),
			math::Vector3::unitZ());
	}

	// ============================================================
	// Yaw
	// ============================================================

	TEST(FrameTransformYawTests, PositiveNinetyDegreeYawMapsForwardToEast)
	{
		/*
		 * Body:
		 * +X = forward
		 *
		 * Navigation NED:
		 * +X = north
		 * +Y = east
		 *
		 * Positive 90 degree yaw therefore points
		 * body-forward toward east.
		 */

		const math::Quaternion qNB =
			math::Quaternion::fromRotationZ(
				PI / 2.0);

		const math::Vector3 result =
			bodyToNavigation(
				math::Vector3::unitX(),
				qNB);

		expectVectorNear(
			result,
			math::Vector3::unitY());
	}

	TEST(FrameTransformYawTests, PositiveNinetyDegreeYawMapsBodyRightToSouth)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationZ(
				PI / 2.0);

		const math::Vector3 result =
			bodyToNavigation(
				math::Vector3::unitY(),
				qNB);

		expectVectorNear(
			result,
			math::Vector3(
				-1.0,
				0.0,
				0.0));
	}

	TEST(FrameTransformYawTests, PositiveNinetyDegreeYawLeavesDownAxisUnchanged)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationZ(
				PI / 2.0);

		expectVectorNear(
			bodyToNavigation(
				math::Vector3::unitZ(),
				qNB),
			math::Vector3::unitZ());
	}

	// ============================================================
	// Roll
	// ============================================================

	TEST(FrameTransformRollTests, PositiveNinetyDegreeRollMapsBodyRightToDown)
	{
		/*
		 * FRD:
		 * +Y = right
		 * +Z = down
		 *
		 * Positive rotation about +X follows
		 * the right-hand rule:
		 *
		 * +Y -> +Z
		 */

		const math::Quaternion qNB =
			math::Quaternion::fromRotationX(
				PI / 2.0);

		expectVectorNear(
			bodyToNavigation(
				math::Vector3::unitY(),
				qNB),
			math::Vector3::unitZ());
	}

	TEST(FrameTransformRollTests, PositiveNinetyDegreeRollMapsBodyDownToNegativeY)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationX(
				PI / 2.0);

		expectVectorNear(
			bodyToNavigation(
				math::Vector3::unitZ(),
				qNB),
			math::Vector3(
				0.0,
				-1.0,
				0.0));
	}

	TEST(FrameTransformRollTests, PositiveRollLeavesForwardAxisUnchanged)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationX(
				PI / 2.0);

		expectVectorNear(
			bodyToNavigation(
				math::Vector3::unitX(),
				qNB),
			math::Vector3::unitX());
	}

	// ============================================================
	// Pitch
	// ============================================================

	TEST(FrameTransformPitchTests, PositiveNinetyDegreePitchMapsForwardToNegativeDown)
	{
		/*
		 * Right-handed rotation about +Y:
		 *
		 * +X -> -Z
		 *
		 * Since +Z is DOWN in NED,
		 * -Z is UP.
		 */

		const math::Quaternion qNB =
			math::Quaternion::fromRotationY(
				PI / 2.0);

		expectVectorNear(
			bodyToNavigation(
				math::Vector3::unitX(),
				qNB),
			math::Vector3(
				0.0,
				0.0,
				-1.0));
	}

	TEST(FrameTransformPitchTests, PositiveNinetyDegreePitchMapsBodyDownToNorth)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationY(
				PI / 2.0);

		expectVectorNear(
			bodyToNavigation(
				math::Vector3::unitZ(),
				qNB),
			math::Vector3::unitX());
	}

	TEST(FrameTransformPitchTests, PositivePitchLeavesBodyRightAxisUnchanged)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromRotationY(
				PI / 2.0);

		expectVectorNear(
			bodyToNavigation(
				math::Vector3::unitY(),
				qNB),
			math::Vector3::unitY());
	}

	// ============================================================
	// Round trips
	// ============================================================

	TEST(FrameTransformRoundTripTests, BodyToNavigationToBodyRestoresVector)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					-3.0),
				1.27);

		const math::Vector3 original(
			4.0,
			-7.0,
			2.5);

		const math::Vector3 navigation =
			bodyToNavigation(
				original,
				qNB);

		const math::Vector3 restored =
			navigationToBody(
				navigation,
				qNB);

		expectVectorNear(
			restored,
			original);
	}

	TEST(FrameTransformRoundTripTests, NavigationToBodyToNavigationRestoresVector)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-2.0,
					5.0,
					1.0),
				0.81);

		const math::Vector3 original(
			8.0,
			3.0,
			-4.0);

		const math::Vector3 body =
			navigationToBody(
				original,
				qNB);

		const math::Vector3 restored =
			bodyToNavigation(
				body,
				qNB);

		expectVectorNear(
			restored,
			original);
	}

	// ============================================================
	// Magnitude preservation
	// ============================================================

	TEST(FrameTransformPropertyTests, BodyToNavigationPreservesMagnitude)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					3.0),
				1.7);

		const math::Vector3 vector(
			10.0,
			-4.0,
			8.0);

		const math::Vector3 transformed =
			bodyToNavigation(
				vector,
				qNB);

		EXPECT_NEAR(
			transformed.magnitude(),
			vector.magnitude(),
			EPSILON);
	}

	TEST(FrameTransformPropertyTests, NavigationToBodyPreservesMagnitude)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					-1.0,
					5.0),
				0.93);

		const math::Vector3 vector(
			7.0,
			2.0,
			-9.0);

		const math::Vector3 transformed =
			navigationToBody(
				vector,
				qNB);

		EXPECT_NEAR(
			transformed.magnitude(),
			vector.magnitude(),
			EPSILON);
	}

	// ============================================================
	// Matrix transforms
	// ============================================================

	TEST(FrameTransformMatrixTests, IdentityBodyToNavigationMatrixIsIdentity)
	{
		expectMatrixNear(
			bodyToNavigationMatrix(
				math::Quaternion::identity()),
			math::Matrix::identity(3));
	}

	TEST(FrameTransformMatrixTests, IdentityNavigationToBodyMatrixIsIdentity)
	{
		expectMatrixNear(
			navigationToBodyMatrix(
				math::Quaternion::identity()),
			math::Matrix::identity(3));
	}

	TEST(FrameTransformMatrixTests, NavigationToBodyMatrixIsTransposeOfBodyToNavigation)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					4.0,
					-2.0),
				1.1);

		const math::Matrix RNB =
			bodyToNavigationMatrix(qNB);

		const math::Matrix RBN =
			navigationToBodyMatrix(qNB);

		expectMatrixNear(
			RBN,
			RNB.transpose());
	}

	TEST(FrameTransformMatrixTests, RotationMatricesAreInverses)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-3.0,
					2.0,
					1.0),
				0.72);

		const math::Matrix RNB =
			bodyToNavigationMatrix(qNB);

		const math::Matrix RBN =
			navigationToBodyMatrix(qNB);

		expectMatrixNear(
			RNB * RBN,
			math::Matrix::identity(3));

		expectMatrixNear(
			RBN * RNB,
			math::Matrix::identity(3));
	}

	TEST(FrameTransformMatrixTests, BodyToNavigationMatrixIsOrthogonal)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					3.0),
				2.1);

		const math::Matrix RNB =
			bodyToNavigationMatrix(qNB);

		expectMatrixNear(
			RNB.transpose() * RNB,
			math::Matrix::identity(3));
	}

	TEST(FrameTransformMatrixTests, NavigationToBodyMatrixIsOrthogonal)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					4.0,
					-2.0,
					1.0),
				1.4);

		const math::Matrix RBN =
			navigationToBodyMatrix(qNB);

		expectMatrixNear(
			RBN.transpose() * RBN,
			math::Matrix::identity(3));
	}

	TEST(FrameTransformMatrixTests, BodyToNavigationMatrixDeterminantIsOne)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					5.0,
					1.0),
				1.8);

		EXPECT_NEAR(
			bodyToNavigationMatrix(qNB)
			.determinant(),
			1.0,
			EPSILON);
	}

	TEST(FrameTransformMatrixTests, NavigationToBodyMatrixDeterminantIsOne)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					5.0,
					1.0),
				1.8);

		EXPECT_NEAR(
			navigationToBodyMatrix(qNB)
			.determinant(),
			1.0,
			EPSILON);
	}

	// ============================================================
	// Quaternion / Matrix consistency
	// ============================================================

	TEST(FrameTransformConsistencyTests, BodyToNavigationMatchesMatrixMultiplication)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					3.0,
					-1.0,
					2.0),
				0.88);

		const math::Vector3 vectorBody(
			5.0,
			-4.0,
			7.0);

		const math::Vector3 quaternionResult =
			bodyToNavigation(
				vectorBody,
				qNB);

		const math::Vector3 matrixResult =
			bodyToNavigationMatrix(qNB) *
			vectorBody;

		expectVectorNear(
			quaternionResult,
			matrixResult);
	}

	TEST(FrameTransformConsistencyTests, NavigationToBodyMatchesMatrixMultiplication)
	{
		const math::Quaternion qNB =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					-1.0,
					3.0,
					4.0),
				1.13);

		const math::Vector3 vectorNavigation(
			-6.0,
			2.0,
			9.0);

		const math::Vector3 quaternionResult =
			navigationToBody(
				vectorNavigation,
				qNB);

		const math::Vector3 matrixResult =
			navigationToBodyMatrix(qNB) *
			vectorNavigation;

		expectVectorNear(
			quaternionResult,
			matrixResult);
	}

	// ============================================================
	// Non-unit quaternion behavior
	// ============================================================

	TEST(FrameTransformPropertyTests, ScaledQuaternionProducesSameBodyToNavigationTransform)
	{
		const math::Quaternion normalized =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					1.0,
					2.0,
					3.0),
				0.67);

		const math::Quaternion scaled =
			normalized * 7.0;

		const math::Vector3 vector(
			3.0,
			-5.0,
			8.0);

		expectVectorNear(
			bodyToNavigation(
				vector,
				scaled),
			bodyToNavigation(
				vector,
				normalized));
	}

	TEST(FrameTransformPropertyTests, ScaledQuaternionProducesSameNavigationToBodyTransform)
	{
		const math::Quaternion normalized =
			math::Quaternion::fromAxisAngle(
				math::Vector3(
					2.0,
					-4.0,
					1.0),
				1.21);

		const math::Quaternion scaled =
			normalized * 12.0;

		const math::Vector3 vector(
			-3.0,
			6.0,
			2.0);

		expectVectorNear(
			navigationToBody(
				vector,
				scaled),
			navigationToBody(
				vector,
				normalized));
	}

	// ============================================================
	// Deterministic randomized verification
	// ============================================================

	TEST(FrameTransformPropertyTests, RandomizedFrameTransformProperties)
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
			math::Vector3 axis(
				componentDistribution(generator),
				componentDistribution(generator),
				componentDistribution(generator));

			if (axis.magnitude() < 1e-6)
			{
				axis =
					math::Vector3::unitX();
			}

			const math::Quaternion qNB =
				math::Quaternion::fromAxisAngle(
					axis,
					angleDistribution(generator));

			const math::Vector3 vectorBody(
				componentDistribution(generator),
				componentDistribution(generator),
				componentDistribution(generator));

			const math::Vector3 vectorNavigation =
				bodyToNavigation(
					vectorBody,
					qNB);

			const math::Vector3 restoredBody =
				navigationToBody(
					vectorNavigation,
					qNB);

			// Body -> Navigation -> Body round trip.
			expectVectorNear(
				restoredBody,
				vectorBody,
				1e-8);

			// Rotation preserves magnitude.
			EXPECT_NEAR(
				vectorNavigation.magnitude(),
				vectorBody.magnitude(),
				1e-8);

			const math::Matrix RNB =
				bodyToNavigationMatrix(qNB);

			const math::Matrix RBN =
				navigationToBodyMatrix(qNB);

			// Quaternion and matrix transforms agree.
			expectVectorNear(
				vectorNavigation,
				RNB * vectorBody,
				1e-8);

			// Reverse rotation is transpose.
			expectMatrixNear(
				RBN,
				RNB.transpose(),
				1e-8);

			// R_BN * R_NB = I.
			expectMatrixNear(
				RBN * RNB,
				math::Matrix::identity(3),
				1e-8);

			// R_NB * R_BN = I.
			expectMatrixNear(
				RNB * RBN,
				math::Matrix::identity(3),
				1e-8);

			// Proper rotations have determinant +1.
			EXPECT_NEAR(
				RNB.determinant(),
				1.0,
				1e-8);

			EXPECT_NEAR(
				RBN.determinant(),
				1.0,
				1e-8);
		}
	}
}