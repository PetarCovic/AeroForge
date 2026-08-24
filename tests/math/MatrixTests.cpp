#include <gtest/gtest.h>

#include <math/Matrix.h>
#include <math/Vector3.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace aeroforge::math::tests
{
	namespace
	{
		void expectMatrixNear(
			const Matrix& actual,
			const Matrix& expected,
			double epsilon = 1e-12)
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
	}

	// ============================================================
	// Construction
	// ============================================================

	TEST(MatrixConstructionTests, DefaultValueConstructorCreatesZeroMatrix)
	{
		Matrix matrix(2, 3);

		EXPECT_EQ(matrix.rows(), 2);
		EXPECT_EQ(matrix.columns(), 3);
		EXPECT_EQ(matrix.size(), 6);

		for (std::size_t row = 0;
			row < matrix.rows();
			++row)
		{
			for (std::size_t column = 0;
				column < matrix.columns();
				++column)
			{
				EXPECT_DOUBLE_EQ(
					matrix(row, column),
					0.0);
			}
		}
	}

	TEST(MatrixConstructionTests, InitialValueConstructorFillsMatrix)
	{
		Matrix matrix(2, 3, 5.0);

		for (std::size_t row = 0;
			row < matrix.rows();
			++row)
		{
			for (std::size_t column = 0;
				column < matrix.columns();
				++column)
			{
				EXPECT_DOUBLE_EQ(
					matrix(row, column),
					5.0);
			}
		}
	}

	TEST(MatrixConstructionTests, FlatInitializerListUsesRowMajorOrder)
	{
		Matrix matrix(
			2,
			3,
			{
				1.0, 2.0, 3.0,
				4.0, 5.0, 6.0
			});

		EXPECT_DOUBLE_EQ(matrix(0, 0), 1.0);
		EXPECT_DOUBLE_EQ(matrix(0, 1), 2.0);
		EXPECT_DOUBLE_EQ(matrix(0, 2), 3.0);
		EXPECT_DOUBLE_EQ(matrix(1, 0), 4.0);
		EXPECT_DOUBLE_EQ(matrix(1, 1), 5.0);
		EXPECT_DOUBLE_EQ(matrix(1, 2), 6.0);
	}

	TEST(MatrixConstructionTests, NestedInitializerListCreatesMatrix)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
		};

		EXPECT_EQ(matrix.rows(), 2);
		EXPECT_EQ(matrix.columns(), 3);

		EXPECT_DOUBLE_EQ(matrix(0, 0), 1.0);
		EXPECT_DOUBLE_EQ(matrix(0, 2), 3.0);
		EXPECT_DOUBLE_EQ(matrix(1, 0), 4.0);
		EXPECT_DOUBLE_EQ(matrix(1, 2), 6.0);
	}

	TEST(MatrixConstructionTests, FlatInitializerRejectsWrongValueCount)
	{
		EXPECT_THROW(
			{
				Matrix matrix(
					2,
					3,
					{1.0, 2.0, 3.0});
			},
			std::invalid_argument);
	}

	TEST(MatrixConstructionTests, NestedInitializerRejectsRaggedRows)
	{
		auto constructRaggedMatrix = []()
			{
				Matrix matrix{
					{1.0, 2.0},
					{3.0}
				};
			};

		EXPECT_THROW(
			constructRaggedMatrix(),
			std::invalid_argument);
	}

	TEST(MatrixConstructionTests, ZeroByZeroMatrixIsEmptyAndSquare)
	{
		Matrix matrix(0, 0);

		EXPECT_EQ(matrix.rows(), 0);
		EXPECT_EQ(matrix.columns(), 0);
		EXPECT_EQ(matrix.size(), 0);
		EXPECT_TRUE(matrix.empty());
		EXPECT_TRUE(matrix.isSquare());
	}

	TEST(MatrixConstructionTests, OverflowingDimensionsThrowLengthError)
	{
		EXPECT_THROW(
			{
				Matrix matrix(
					std::numeric_limits<std::size_t>::max(),
					2);
			},
			std::length_error);
	}

	// ============================================================
	// Value access
	// ============================================================

	TEST(MatrixAccessTests, OperatorParenthesesCanReadAndWriteValues)
	{
		Matrix matrix(2, 2);

		matrix(0, 0) = 1.0;
		matrix(0, 1) = 2.0;
		matrix(1, 0) = 3.0;
		matrix(1, 1) = 4.0;

		EXPECT_DOUBLE_EQ(matrix(0, 0), 1.0);
		EXPECT_DOUBLE_EQ(matrix(0, 1), 2.0);
		EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
		EXPECT_DOUBLE_EQ(matrix(1, 1), 4.0);
	}

	TEST(MatrixAccessTests, ConstOperatorParenthesesReadsValue)
	{
		const Matrix matrix{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		EXPECT_DOUBLE_EQ(matrix(1, 0), 3.0);
	}

	TEST(MatrixAccessTests, SetValueAndGetValueWork)
	{
		Matrix matrix(2, 2);

		Matrix& result =
			matrix.setValue(1, 0, 7.5);

		EXPECT_EQ(&result, &matrix);
		EXPECT_DOUBLE_EQ(
			matrix.getValue(1, 0),
			7.5);
	}

	TEST(MatrixAccessTests, OutOfRangeAccessThrows)
	{
		Matrix matrix(2, 3);

		EXPECT_THROW(
			matrix(2, 0),
			std::out_of_range);

		EXPECT_THROW(
			matrix(0, 3),
			std::out_of_range);

		EXPECT_THROW(
			matrix.getValue(10, 10),
			std::out_of_range);
	}

	// ============================================================
	// setValues / fill
	// ============================================================

	TEST(MatrixValueTests, FlatSetValuesUsesRowMajorOrder)
	{
		Matrix matrix(2, 2);

		matrix.setValues({
			1.0, 2.0,
			3.0, 4.0
			});

		Matrix expected{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		expectMatrixNear(matrix, expected);
	}

	TEST(MatrixValueTests, NestedSetValuesWorks)
	{
		Matrix matrix(2, 3);

		matrix.setValues({
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
			});

		Matrix expected{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
		};

		expectMatrixNear(matrix, expected);
	}

	TEST(MatrixValueTests, FlatSetValuesRejectsWrongCount)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.setValues({
				1.0,
				2.0,
				3.0
				}),
			std::invalid_argument);
	}

	TEST(MatrixValueTests, NestedSetValuesRejectsWrongRowCount)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.setValues({
				{1.0, 2.0}
				}),
			std::invalid_argument);
	}

	TEST(MatrixValueTests, NestedSetValuesRejectsWrongColumnCount)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.setValues({
				{1.0, 2.0},
				{3.0}
				}),
			std::invalid_argument);
	}

	TEST(MatrixValueTests, FillSetsEveryEntry)
	{
		Matrix matrix{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix& result =
			matrix.fill(-3.5);

		EXPECT_EQ(&result, &matrix);

		for (std::size_t row = 0;
			row < matrix.rows();
			++row)
		{
			for (std::size_t column = 0;
				column < matrix.columns();
				++column)
			{
				EXPECT_DOUBLE_EQ(
					matrix(row, column),
					-3.5);
			}
		}
	}

	// ============================================================
	// Factories
	// ============================================================

	TEST(MatrixFactoryTests, IdentityCreatesIdentityMatrix)
	{
		Matrix actual =
			Matrix::identity(4);

		Matrix expected{
			{1.0, 0.0, 0.0, 0.0},
			{0.0, 1.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		};

		expectMatrixNear(actual, expected);
	}

	TEST(MatrixFactoryTests, ZerosCreatesZeroMatrix)
	{
		Matrix matrix =
			Matrix::zeros(2, 3);

		EXPECT_EQ(matrix.rows(), 2);
		EXPECT_EQ(matrix.columns(), 3);

		for (std::size_t row = 0;
			row < 2;
			++row)
		{
			for (std::size_t column = 0;
				column < 3;
				++column)
			{
				EXPECT_DOUBLE_EQ(
					matrix(row, column),
					0.0);
			}
		}
	}

	TEST(MatrixFactoryTests, OnesCreatesOnesMatrix)
	{
		Matrix matrix =
			Matrix::ones(3, 2);

		for (std::size_t row = 0;
			row < 3;
			++row)
		{
			for (std::size_t column = 0;
				column < 2;
				++column)
			{
				EXPECT_DOUBLE_EQ(
					matrix(row, column),
					1.0);
			}
		}
	}

	TEST(MatrixFactoryTests, DiagonalCreatesCorrectMatrix)
	{
		Matrix actual =
			Matrix::diagonal(4.0, 3);

		Matrix expected{
			{4.0, 0.0, 0.0},
			{0.0, 4.0, 0.0},
			{0.0, 0.0, 4.0}
		};

		expectMatrixNear(actual, expected);
	}

	TEST(MatrixFactoryTests, SeededRandomIsDeterministic)
	{
		Matrix first =
			Matrix::random(5, 4, 123456);

		Matrix second =
			Matrix::random(5, 4, 123456);

		expectMatrixNear(
			first,
			second,
			0.0);
	}

	TEST(MatrixFactoryTests, RandomValuesAreWithinExpectedRange)
	{
		Matrix matrix =
			Matrix::random(10, 10, 42);

		for (std::size_t row = 0;
			row < matrix.rows();
			++row)
		{
			for (std::size_t column = 0;
				column < matrix.columns();
				++column)
			{
				EXPECT_GE(
					matrix(row, column),
					0.0);

				EXPECT_LT(
					matrix(row, column),
					1.0);
			}
		}
	}

	// ============================================================
	// Dimensions
	// ============================================================

	TEST(MatrixDimensionTests, ReportsDimensionsCorrectly)
	{
		Matrix matrix(4, 7);

		EXPECT_EQ(matrix.rows(), 4);
		EXPECT_EQ(matrix.columns(), 7);
		EXPECT_EQ(matrix.size(), 28);
		EXPECT_FALSE(matrix.empty());
		EXPECT_FALSE(matrix.isSquare());
	}

	TEST(MatrixDimensionTests, SquareMatrixIsDetected)
	{
		Matrix matrix(5, 5);

		EXPECT_TRUE(matrix.isSquare());
	}

	// ============================================================
	// Addition
	// ============================================================

	TEST(MatrixAdditionTests, AddWorks)
	{
		Matrix a{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix b{
			{5.0, 6.0},
			{7.0, 8.0}
		};

		Matrix expected{
			{6.0, 8.0},
			{10.0, 12.0}
		};

		expectMatrixNear(
			a.add(b),
			expected);
	}

	TEST(MatrixAdditionTests, PlusOperatorWorks)
	{
		Matrix a{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix b{
			{5.0, 6.0},
			{7.0, 8.0}
		};

		Matrix expected{
			{6.0, 8.0},
			{10.0, 12.0}
		};

		expectMatrixNear(
			a + b,
			expected);
	}

	TEST(MatrixAdditionTests, PlusEqualsWorks)
	{
		Matrix a{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix b{
			{5.0, 6.0},
			{7.0, 8.0}
		};

		a += b;

		Matrix expected{
			{6.0, 8.0},
			{10.0, 12.0}
		};

		expectMatrixNear(a, expected);
	}

	TEST(MatrixAdditionTests, MismatchedDimensionsThrow)
	{
		Matrix a(2, 3);
		Matrix b(3, 2);

		EXPECT_THROW(
			a + b,
			std::invalid_argument);

		EXPECT_THROW(
			a.add(b),
			std::invalid_argument);

		EXPECT_THROW(
			a += b,
			std::invalid_argument);
	}

	// ============================================================
	// Subtraction
	// ============================================================

	TEST(MatrixSubtractionTests, SubWorks)
	{
		Matrix a{
			{6.0, 8.0},
			{10.0, 12.0}
		};

		Matrix b{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix expected{
			{5.0, 6.0},
			{7.0, 8.0}
		};

		expectMatrixNear(
			a.sub(b),
			expected);
	}

	TEST(MatrixSubtractionTests, MinusOperatorWorks)
	{
		Matrix a{
			{6.0, 8.0},
			{10.0, 12.0}
		};

		Matrix b{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix expected{
			{5.0, 6.0},
			{7.0, 8.0}
		};

		expectMatrixNear(
			a - b,
			expected);
	}

	TEST(MatrixSubtractionTests, MinusEqualsWorks)
	{
		Matrix a{
			{6.0, 8.0},
			{10.0, 12.0}
		};

		Matrix b{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		a -= b;

		Matrix expected{
			{5.0, 6.0},
			{7.0, 8.0}
		};

		expectMatrixNear(a, expected);
	}

	TEST(MatrixSubtractionTests, MismatchedDimensionsThrow)
	{
		Matrix a(2, 2);
		Matrix b(2, 3);

		EXPECT_THROW(
			a - b,
			std::invalid_argument);

		EXPECT_THROW(
			a.sub(b),
			std::invalid_argument);

		EXPECT_THROW(
			a -= b,
			std::invalid_argument);
	}

	// ============================================================
	// Scalar multiplication
	// ============================================================

	TEST(MatrixScalarMultiplicationTests, ScaleWorks)
	{
		Matrix matrix{
			{1.0, -2.0},
			{3.0, 4.0}
		};

		Matrix expected{
			{2.0, -4.0},
			{6.0, 8.0}
		};

		expectMatrixNear(
			matrix.scale(2.0),
			expected);
	}

	TEST(MatrixScalarMultiplicationTests, MatrixTimesScalarWorks)
	{
		Matrix matrix{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix expected{
			{3.0, 6.0},
			{9.0, 12.0}
		};

		expectMatrixNear(
			matrix * 3.0,
			expected);
	}

	TEST(MatrixScalarMultiplicationTests, ScalarTimesMatrixWorks)
	{
		Matrix matrix{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix expected{
			{3.0, 6.0},
			{9.0, 12.0}
		};

		expectMatrixNear(
			3.0 * matrix,
			expected);
	}

	TEST(MatrixScalarMultiplicationTests, TimesEqualsWorks)
	{
		Matrix matrix{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		matrix *= -2.0;

		Matrix expected{
			{-2.0, -4.0},
			{-6.0, -8.0}
		};

		expectMatrixNear(matrix, expected);
	}

	// ============================================================
	// Scalar division
	// ============================================================

	TEST(MatrixScalarDivisionTests, DivWorks)
	{
		Matrix matrix{
			{2.0, 4.0},
			{6.0, 8.0}
		};

		Matrix expected{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		expectMatrixNear(
			matrix.div(2.0),
			expected);
	}

	TEST(MatrixScalarDivisionTests, DivisionOperatorWorks)
	{
		Matrix matrix{
			{2.0, 4.0},
			{6.0, 8.0}
		};

		Matrix expected{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		expectMatrixNear(
			matrix / 2.0,
			expected);
	}

	TEST(MatrixScalarDivisionTests, DivisionEqualsWorks)
	{
		Matrix matrix{
			{2.0, 4.0},
			{6.0, 8.0}
		};

		matrix /= 2.0;

		Matrix expected{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		expectMatrixNear(matrix, expected);
	}

	TEST(MatrixScalarDivisionTests, DivisionByZeroThrows)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.div(0.0),
			std::invalid_argument);

		EXPECT_THROW(
			matrix / 0.0,
			std::invalid_argument);

		EXPECT_THROW(
			matrix /= 0.0,
			std::invalid_argument);
	}

	// ============================================================
	// Matrix multiplication
	// ============================================================

	TEST(MatrixMultiplicationTests, RectangularMultiplicationWorks)
	{
		Matrix a{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
		};

		Matrix b{
			{7.0, 8.0},
			{9.0, 10.0},
			{11.0, 12.0}
		};

		Matrix expected{
			{58.0, 64.0},
			{139.0, 154.0}
		};

		expectMatrixNear(
			a.mult(b),
			expected);

		expectMatrixNear(
			a * b,
			expected);
	}

	TEST(MatrixMultiplicationTests, MultiplicationProducesCorrectDimensions)
	{
		Matrix a(4, 7);
		Matrix b(7, 3);

		Matrix result = a * b;

		EXPECT_EQ(result.rows(), 4);
		EXPECT_EQ(result.columns(), 3);
	}

	TEST(MatrixMultiplicationTests, IdentityDoesNotChangeMatrix)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0},
			{7.0, 8.0, 9.0}
		};

		Matrix identity =
			Matrix::identity(3);

		expectMatrixNear(
			matrix * identity,
			matrix);

		expectMatrixNear(
			identity * matrix,
			matrix);
	}

	TEST(MatrixMultiplicationTests, IncompatibleDimensionsThrow)
	{
		Matrix a(2, 3);
		Matrix b(2, 2);

		EXPECT_THROW(
			a * b,
			std::invalid_argument);

		EXPECT_THROW(
			a.mult(b),
			std::invalid_argument);
	}

	// ============================================================
	// Matrix x Vector3
	// ============================================================

	TEST(MatrixVectorTests, MatrixVectorMultiplicationWorks)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0},
			{7.0, 8.0, 9.0}
		};

		Vector3 vector(
			1.0,
			2.0,
			3.0);

		Vector3 result =
			matrix * vector;

		EXPECT_DOUBLE_EQ(result.x, 14.0);
		EXPECT_DOUBLE_EQ(result.y, 32.0);
		EXPECT_DOUBLE_EQ(result.z, 50.0);
	}

	TEST(MatrixVectorTests, MultVectorWorks)
	{
		Matrix matrix{
			{2.0, 0.0, 0.0},
			{0.0, 3.0, 0.0},
			{0.0, 0.0, 4.0}
		};

		Vector3 result =
			matrix.multVector(
				Vector3(1.0, 2.0, 3.0));

		EXPECT_DOUBLE_EQ(result.x, 2.0);
		EXPECT_DOUBLE_EQ(result.y, 6.0);
		EXPECT_DOUBLE_EQ(result.z, 12.0);
	}

	TEST(MatrixVectorTests, NonThreeByThreeMatrixThrows)
	{
		Matrix matrix(4, 4);
		Vector3 vector(1.0, 2.0, 3.0);

		EXPECT_THROW(
			matrix * vector,
			std::invalid_argument);
	}

	TEST(MatrixVectorTests, VectorConvertsToThreeByOneMatrix)
	{
		Vector3 vector(
			1.5,
			-2.0,
			3.25);

		Matrix matrix =
			toMatrix(vector);

		EXPECT_EQ(matrix.rows(), 3);
		EXPECT_EQ(matrix.columns(), 1);

		EXPECT_DOUBLE_EQ(matrix(0, 0), 1.5);
		EXPECT_DOUBLE_EQ(matrix(1, 0), -2.0);
		EXPECT_DOUBLE_EQ(matrix(2, 0), 3.25);
	}

	TEST(MatrixVectorTests, ThreeByOneMatrixConvertsToVector)
	{
		Matrix matrix{
			{1.5},
			{-2.0},
			{3.25}
		};

		Vector3 vector =
			toVector3(matrix);

		EXPECT_DOUBLE_EQ(vector.x, 1.5);
		EXPECT_DOUBLE_EQ(vector.y, -2.0);
		EXPECT_DOUBLE_EQ(vector.z, 3.25);
	}

	TEST(MatrixVectorTests, InvalidMatrixCannotConvertToVector)
	{
		Matrix matrix(1, 3);

		EXPECT_THROW(
			toVector3(matrix),
			std::invalid_argument);
	}

	// ============================================================
	// Transpose
	// ============================================================

	TEST(MatrixTransposeTests, RectangularTransposeWorks)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
		};

		Matrix expected{
			{1.0, 4.0},
			{2.0, 5.0},
			{3.0, 6.0}
		};

		expectMatrixNear(
			matrix.transpose(),
			expected);
	}

	TEST(MatrixTransposeTests, DoubleTransposeReturnsOriginal)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
		};

		expectMatrixNear(
			matrix.transpose().transpose(),
			matrix);
	}

	// ============================================================
	// Trace
	// ============================================================

	TEST(MatrixTraceTests, TraceWorks)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0},
			{7.0, 8.0, 9.0}
		};

		EXPECT_DOUBLE_EQ(
			matrix.trace(),
			15.0);
	}

	TEST(MatrixTraceTests, NonSquareTraceThrows)
	{
		Matrix matrix(2, 3);

		EXPECT_THROW(
			matrix.trace(),
			std::domain_error);
	}

	// ============================================================
	// Determinant
	// ============================================================

	TEST(MatrixDeterminantTests, EmptyMatrixHasDeterminantOne)
	{
		Matrix matrix(0, 0);

		EXPECT_DOUBLE_EQ(
			matrix.determinant(),
			1.0);
	}

	TEST(MatrixDeterminantTests, OneByOneDeterminantWorks)
	{
		Matrix matrix{
			{7.0}
		};

		EXPECT_DOUBLE_EQ(
			matrix.determinant(),
			7.0);
	}

	TEST(MatrixDeterminantTests, TwoByTwoDeterminantWorks)
	{
		Matrix matrix{
			{4.0, 7.0},
			{2.0, 6.0}
		};

		EXPECT_NEAR(
			matrix.determinant(),
			10.0,
			1e-12);
	}

	TEST(MatrixDeterminantTests, ThreeByThreeDeterminantWorks)
	{
		Matrix matrix{
			{6.0, 1.0, 1.0},
			{4.0, -2.0, 5.0},
			{2.0, 8.0, 7.0}
		};

		EXPECT_NEAR(
			matrix.determinant(),
			-306.0,
			1e-10);
	}

	TEST(MatrixDeterminantTests, RowSwapRequiredForPivot)
	{
		Matrix matrix{
			{0.0, 2.0},
			{3.0, 4.0}
		};

		EXPECT_NEAR(
			matrix.determinant(),
			-6.0,
			1e-12);
	}

	TEST(MatrixDeterminantTests, SingularMatrixReturnsZero)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{2.0, 4.0, 6.0},
			{1.0, 1.0, 1.0}
		};

		EXPECT_NEAR(
			matrix.determinant(),
			0.0,
			1e-12);
	}

	TEST(MatrixDeterminantTests, ZeroPivotColumnReturnsZero)
	{
		Matrix matrix{
			{1.0, 0.0, 0.0},
			{0.0, 0.0, 1.0},
			{0.0, 0.0, 2.0}
		};

		EXPECT_DOUBLE_EQ(
			matrix.determinant(),
			0.0);
	}

	TEST(MatrixDeterminantTests, VeryDifferentScalesRemainValid)
	{
		Matrix matrix{
			{1e-20, 0.0},
			{0.0, 1e20}
		};

		EXPECT_NEAR(
			matrix.determinant(),
			1.0,
			1e-12);
	}

	TEST(MatrixDeterminantTests, NonSquareDeterminantThrows)
	{
		Matrix matrix(2, 3);

		EXPECT_THROW(
			matrix.determinant(),
			std::domain_error);
	}

	// ============================================================
	// Cofactor / adjoint
	// ============================================================

	TEST(MatrixCofactorTests, CofactorWorks)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{0.0, 4.0, 5.0},
			{1.0, 0.0, 6.0}
		};

		Matrix expected{
			{24.0, 5.0, -4.0},
			{-12.0, 3.0, 2.0},
			{-2.0, -5.0, 4.0}
		};

		expectMatrixNear(
			matrix.cofactor(),
			expected,
			1e-10);
	}

	TEST(MatrixCofactorTests, OneByOneCofactorIsOne)
	{
		Matrix matrix{
			{5.0}
		};

		Matrix expected{
			{1.0}
		};

		expectMatrixNear(
			matrix.cofactor(),
			expected);
	}

	TEST(MatrixCofactorTests, NonSquareCofactorThrows)
	{
		Matrix matrix(2, 3);

		EXPECT_THROW(
			matrix.cofactor(),
			std::domain_error);
	}

	TEST(MatrixAdjointTests, AdjointWorks)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{0.0, 4.0, 5.0},
			{1.0, 0.0, 6.0}
		};

		Matrix expected{
			{24.0, -12.0, -2.0},
			{5.0, 3.0, -5.0},
			{-4.0, 2.0, 4.0}
		};

		expectMatrixNear(
			matrix.adjoint(),
			expected,
			1e-10);
	}

	TEST(MatrixAdjointTests, NonSquareAdjointThrows)
	{
		Matrix matrix(3, 2);

		EXPECT_THROW(
			matrix.adjoint(),
			std::domain_error);
	}

	// ============================================================
	// Inverse
	// ============================================================

	TEST(MatrixInverseTests, TwoByTwoInverseWorks)
	{
		Matrix matrix{
			{4.0, 7.0},
			{2.0, 6.0}
		};

		Matrix expected{
			{0.6, -0.7},
			{-0.2, 0.4}
		};

		expectMatrixNear(
			matrix.inverse(),
			expected,
			1e-12);
	}

	TEST(MatrixInverseTests, OneByOneInverseWorks)
	{
		Matrix matrix{
			{4.0}
		};

		Matrix expected{
			{0.25}
		};

		expectMatrixNear(
			matrix.inverse(),
			expected);
	}

	TEST(MatrixInverseTests, IdentityInverseIsIdentity)
	{
		Matrix identity =
			Matrix::identity(5);

		expectMatrixNear(
			identity.inverse(),
			identity);
	}

	TEST(MatrixInverseTests, InverseHandlesRequiredRowSwap)
	{
		Matrix matrix{
			{0.0, 1.0},
			{2.0, 3.0}
		};

		Matrix inverse =
			matrix.inverse();

		Matrix product =
			matrix * inverse;

		expectMatrixNear(
			product,
			Matrix::identity(2),
			1e-12);
	}

	TEST(MatrixInverseTests, MatrixTimesInverseProducesIdentity)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{0.0, 4.0, 5.0},
			{1.0, 0.0, 6.0}
		};

		Matrix inverse =
			matrix.inverse();

		Matrix product =
			matrix * inverse;

		expectMatrixNear(
			product,
			Matrix::identity(3),
			1e-10);
	}

	TEST(MatrixInverseTests, InverseTimesMatrixProducesIdentity)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{0.0, 4.0, 5.0},
			{1.0, 0.0, 6.0}
		};

		Matrix product =
			matrix.inverse() * matrix;

		expectMatrixNear(
			product,
			Matrix::identity(3),
			1e-10);
	}

	TEST(MatrixInverseTests, BadlyScaledInvertibleMatrixWorks)
	{
		Matrix matrix{
			{1e20, 0.0},
			{0.0, 1.0}
		};

		Matrix expected{
			{1e-20, 0.0},
			{0.0, 1.0}
		};

		expectMatrixNear(
			matrix.inverse(),
			expected,
			1e-15);
	}

	TEST(MatrixInverseTests, SingularMatrixThrows)
	{
		Matrix matrix{
			{1.0, 2.0},
			{2.0, 4.0}
		};

		EXPECT_THROW(
			matrix.inverse(),
			std::domain_error);
	}

	TEST(MatrixInverseTests, NonSquareMatrixThrows)
	{
		Matrix matrix(2, 3);

		EXPECT_THROW(
			matrix.inverse(),
			std::domain_error);
	}

	TEST(MatrixInverseTests, EmptyMatrixInverseIsEmpty)
	{
		Matrix matrix(0, 0);

		Matrix inverse =
			matrix.inverse();

		EXPECT_EQ(inverse.rows(), 0);
		EXPECT_EQ(inverse.columns(), 0);
		EXPECT_TRUE(inverse.empty());
	}

	TEST(MatrixInverseTests, GeneralFifteenByFifteenInverseProducesIdentity)
	{
		Matrix matrix =
			Matrix::random(
				15,
				15,
				123456789);

		// Make it strongly diagonally dominant and
		// therefore safely invertible for this test.
		for (std::size_t i = 0;
			i < 15;
			++i)
		{
			matrix(i, i) += 15.0;
		}

		Matrix inverse =
			matrix.inverse();

		Matrix product =
			matrix * inverse;

		expectMatrixNear(
			product,
			Matrix::identity(15),
			1e-9);
	}

	// ============================================================
	// Finite checks
	// ============================================================

	TEST(MatrixFiniteTests, FiniteMatrixReturnsTrue)
	{
		Matrix matrix{
			{1.0, -2.0},
			{3.0, 4.0}
		};

		EXPECT_TRUE(matrix.isFinite());
	}

	TEST(MatrixFiniteTests, NaNReturnsFalse)
	{
		Matrix matrix{
			{1.0, 2.0},
			{3.0, std::numeric_limits<double>::quiet_NaN()}
		};

		EXPECT_FALSE(matrix.isFinite());
	}

	TEST(MatrixFiniteTests, InfinityReturnsFalse)
	{
		Matrix matrix{
			{1.0, 2.0},
			{3.0, std::numeric_limits<double>::infinity()}
		};

		EXPECT_FALSE(matrix.isFinite());
	}

	// ============================================================
	// Approximate equality
	// ============================================================

	TEST(MatrixApproxTests, IdenticalMatricesAreApproximate)
	{
		Matrix a{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix b = a;

		EXPECT_TRUE(
			a.isApprox(b, 1e-9));
	}

	TEST(MatrixApproxTests, SmallAbsoluteDifferenceIsApproximate)
	{
		Matrix a{
			{1.0}
		};

		Matrix b{
			{1.0 + 1e-10}
		};

		EXPECT_TRUE(
			a.isApprox(b, 1e-9));
	}

	TEST(MatrixApproxTests, LargeValuesUseRelativeTolerance)
	{
		Matrix a{
			{1e12}
		};

		Matrix b{
			{1e12 + 100.0}
		};

		EXPECT_TRUE(
			a.isApprox(b, 1e-9));
	}

	TEST(MatrixApproxTests, DifferenceOutsideToleranceReturnsFalse)
	{
		Matrix a{
			{1.0}
		};

		Matrix b{
			{1.1}
		};

		EXPECT_FALSE(
			a.isApprox(b, 1e-6));
	}

	TEST(MatrixApproxTests, DifferentDimensionsReturnFalse)
	{
		Matrix a(2, 2);
		Matrix b(2, 3);

		EXPECT_FALSE(
			a.isApprox(b, 1e-9));
	}

	TEST(MatrixApproxTests, ZeroEpsilonThrows)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.isApprox(matrix, 0.0),
			std::invalid_argument);
	}

	TEST(MatrixApproxTests, NegativeEpsilonThrows)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.isApprox(matrix, -1.0),
			std::invalid_argument);
	}

	TEST(MatrixApproxTests, NaNMatricesAreNotApproximate)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		Matrix a{
			{nan}
		};

		Matrix b{
			{nan}
		};

		EXPECT_FALSE(
			a.isApprox(b, 1e-9));
	}

	TEST(MatrixApproxTests, InfiniteMatricesAreNotApproximate)
	{
		const double infinity =
			std::numeric_limits<double>::infinity();

		Matrix a{
			{infinity}
		};

		Matrix b{
			{infinity}
		};

		EXPECT_FALSE(
			a.isApprox(b, 1e-9));
	}

	// ============================================================
	// Row swaps
	// ============================================================

	TEST(MatrixRowSwapTests, SwapRowsWorks)
	{
		Matrix matrix{
			{1.0, 2.0},
			{3.0, 4.0},
			{5.0, 6.0}
		};

		Matrix result =
			matrix.swapRows(0, 2);

		Matrix expected{
			{5.0, 6.0},
			{3.0, 4.0},
			{1.0, 2.0}
		};

		expectMatrixNear(result, expected);

		// Original must remain unchanged.
		EXPECT_DOUBLE_EQ(
			matrix(0, 0),
			1.0);
	}

	TEST(MatrixRowSwapTests, InvalidRowThrows)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.swapRows(0, 2),
			std::out_of_range);
	}

	// ============================================================
	// Column swaps
	// ============================================================

	TEST(MatrixColumnSwapTests, SwapColumnsWorks)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
		};

		Matrix result =
			matrix.swapColumns(0, 2);

		Matrix expected{
			{3.0, 2.0, 1.0},
			{6.0, 5.0, 4.0}
		};

		expectMatrixNear(result, expected);

		EXPECT_DOUBLE_EQ(
			matrix(0, 0),
			1.0);
	}

	TEST(MatrixColumnSwapTests, InvalidColumnThrows)
	{
		Matrix matrix(2, 2);

		EXPECT_THROW(
			matrix.swapColumns(0, 2),
			std::out_of_range);
	}

	// ============================================================
	// Augment
	// ============================================================

	TEST(MatrixAugmentTests, AugmentWorks)
	{
		Matrix left{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix right{
			{5.0},
			{6.0}
		};

		Matrix expected{
			{1.0, 2.0, 5.0},
			{3.0, 4.0, 6.0}
		};

		expectMatrixNear(
			left.augment(right),
			expected);
	}

	TEST(MatrixAugmentTests, DifferentRowCountsThrow)
	{
		Matrix a(2, 2);
		Matrix b(3, 1);

		EXPECT_THROW(
			a.augment(b),
			std::invalid_argument);
	}

	// ============================================================
	// Submatrix
	// ============================================================

	TEST(MatrixSubMatrixTests, ExtractsCorrectSubMatrix)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0, 4.0},
			{5.0, 6.0, 7.0, 8.0},
			{9.0, 10.0, 11.0, 12.0}
		};

		Matrix expected{
			{6.0, 7.0},
			{10.0, 11.0}
		};

		expectMatrixNear(
			matrix.subMatrix(
				1,
				1,
				2,
				2),
			expected);
	}

	TEST(MatrixSubMatrixTests, InvalidStartRowThrows)
	{
		Matrix matrix(3, 3);

		EXPECT_THROW(
			matrix.subMatrix(
				3,
				0,
				1,
				1),
			std::out_of_range);
	}

	TEST(MatrixSubMatrixTests, InvalidStartColumnThrows)
	{
		Matrix matrix(3, 3);

		EXPECT_THROW(
			matrix.subMatrix(
				0,
				3,
				1,
				1),
			std::out_of_range);
	}

	TEST(MatrixSubMatrixTests, ZeroRowCountThrows)
	{
		Matrix matrix(3, 3);

		EXPECT_THROW(
			matrix.subMatrix(
				0,
				0,
				0,
				1),
			std::invalid_argument);
	}

	TEST(MatrixSubMatrixTests, ZeroColumnCountThrows)
	{
		Matrix matrix(3, 3);

		EXPECT_THROW(
			matrix.subMatrix(
				0,
				0,
				1,
				0),
			std::invalid_argument);
	}

	TEST(MatrixSubMatrixTests, RegionPastMatrixBoundsThrows)
	{
		Matrix matrix(3, 3);

		EXPECT_THROW(
			matrix.subMatrix(
				1,
				1,
				3,
				3),
			std::out_of_range);
	}

	// ============================================================
	// Matrix algebra identities
	// ============================================================

	TEST(MatrixAlgebraTests, TransposeOfProductMatchesReverseProduct)
	{
		Matrix a{
			{1.0, 2.0, 3.0},
			{4.0, 5.0, 6.0}
		};

		Matrix b{
			{7.0, 8.0},
			{9.0, 10.0},
			{11.0, 12.0}
		};

		Matrix left =
			(a * b).transpose();

		Matrix right =
			b.transpose() * a.transpose();

		expectMatrixNear(
			left,
			right,
			1e-12);
	}

	TEST(MatrixAlgebraTests, DeterminantOfProductEqualsProductOfDeterminants)
	{
		Matrix a{
			{2.0, 1.0, 3.0},
			{0.0, 4.0, 2.0},
			{1.0, 0.0, 5.0}
		};

		Matrix b{
			{3.0, 2.0, 0.0},
			{1.0, 1.0, 4.0},
			{2.0, 0.0, 1.0}
		};

		const double left =
			(a * b).determinant();

		const double right =
			a.determinant()
			* b.determinant();

		EXPECT_NEAR(
			left,
			right,
			1e-9);
	}

	TEST(MatrixAlgebraTests, MultiplicationDistributesOverAddition)
	{
		Matrix a{
			{1.0, 2.0},
			{3.0, 4.0}
		};

		Matrix b{
			{5.0, 6.0},
			{7.0, 8.0}
		};

		Matrix c{
			{2.0, 1.0},
			{0.0, 3.0}
		};

		Matrix left =
			a * (b + c);

		Matrix right =
			(a * b) + (a * c);

		expectMatrixNear(
			left,
			right,
			1e-12);
	}

	TEST(MatrixAlgebraTests, AdjointIdentityHolds)
	{
		Matrix matrix{
			{1.0, 2.0, 3.0},
			{0.0, 4.0, 5.0},
			{1.0, 0.0, 6.0}
		};

		Matrix left =
			matrix * matrix.adjoint();

		Matrix right =
			Matrix::identity(3)
			* matrix.determinant();

		expectMatrixNear(
			left,
			right,
			1e-9);
	}
}