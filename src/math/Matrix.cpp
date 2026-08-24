#include <math/Matrix.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace aeroforge::math
{
	Matrix::Matrix(
		std::size_t rows,
		std::size_t columns)
		: Rows(rows),
		Columns(columns),
		m(checkedElementCount(rows, columns), 0.0)
	{
	}

	Matrix::Matrix(
		std::size_t rows,
		std::size_t columns,
		double initialValue)
		: Rows(rows),
		Columns(columns),
		m(checkedElementCount(rows, columns), initialValue)
	{
	}

	Matrix::Matrix(
		std::size_t rows,
		std::size_t columns,
		std::initializer_list<double> values)
		: Rows(rows),
		Columns(columns),
		m(values)
	{
		const std::size_t requiredSize =checkedElementCount(rows, columns);

		if (values.size() != requiredSize)
		{
			throw std::invalid_argument(
				"Number of matrix values does not match matrix dimensions");
		}
	}

	Matrix::Matrix(
		std::initializer_list<
		std::initializer_list<double>> values)
		: Rows(values.size()),
		Columns(values.size() == 0
			? 0
			: values.begin()->size()),
		m(checkedElementCount(Rows, Columns), 0.0)
	{
		std::size_t row = 0;

		for (const auto& rowValues : values)
		{
			if (rowValues.size() != Columns)
			{
				throw std::invalid_argument(
					"All matrix rows must contain the same number of columns");
			}

			std::size_t column = 0;

			for (double value : rowValues)
			{
				(*this)(row, column) = value;
				++column;
			}

			++row;
		}
	}

	std::size_t Matrix::index(
		std::size_t row,
		std::size_t column) const
	{
		if (row >= Rows || column >= Columns)
		{
			throw std::out_of_range(
				"Matrix index out of range");
		}

		return row * Columns + column;
	}

	double& Matrix::operator()(
		std::size_t row,
		std::size_t column)
	{
		return m[index(row, column)];
	}

	const double& Matrix::operator()(
		std::size_t row,
		std::size_t column) const
	{
		return m[index(row, column)];
	}

	Matrix& Matrix::setValue(
		std::size_t row,
		std::size_t column,
		double value)
	{
		(*this)(row, column) = value;
		return *this;
	}

	double Matrix::getValue(
		std::size_t row,
		std::size_t column) const
	{
		return (*this)(row, column);
	}

	Matrix& Matrix::setValues(
		std::initializer_list<double> values)
	{
		if (values.size() != m.size())
		{
			throw std::invalid_argument(
				"Number of values does not match matrix dimensions");
		}

		std::copy(
			values.begin(),
			values.end(),
			m.begin());

		return *this;
	}

	Matrix& Matrix::setValues(
		std::initializer_list<
		std::initializer_list<double>> values)
	{
		if (values.size() != Rows)
		{
			throw std::invalid_argument(
				"Number of rows does not match matrix dimensions");
		}

		std::size_t row = 0;

		for (const auto& rowValues : values)
		{
			if (rowValues.size() != Columns)
			{
				throw std::invalid_argument(
					"Number of columns does not match matrix dimensions");
			}

			std::size_t column = 0;

			for (double value : rowValues)
			{
				(*this)(row, column) = value;
				++column;
			}

			++row;
		}

		return *this;
	}

	Matrix& Matrix::fill(double value)
	{
		std::fill(
			m.begin(),
			m.end(),
			value);

		return *this;
	}

	Matrix Matrix::identity(std::size_t size)
	{
		Matrix result(size, size);

		for (std::size_t i = 0; i < size; ++i)
		{
			result(i, i) = 1.0;
		}

		return result;
	}

	Matrix Matrix::zeros(
		std::size_t rows,
		std::size_t columns)
	{
		return Matrix(rows, columns, 0.0);
	}

	Matrix Matrix::ones(
		std::size_t rows,
		std::size_t columns)
	{
		return Matrix(rows, columns, 1.0);
	}

	Matrix Matrix::random(
		std::size_t rows,
		std::size_t columns)
	{
		std::random_device device;

		const std::uint64_t seed =
			(static_cast<std::uint64_t>(device()) << 32)
			^ static_cast<std::uint64_t>(device());

		return random(rows, columns, seed);
	}

	Matrix Matrix::random(
		std::size_t rows,
		std::size_t columns,
		std::uint64_t seed)
	{
		Matrix result(rows, columns);

		std::mt19937_64 generator(seed);
		std::uniform_real_distribution<double>
			distribution(0.0, 1.0);

		for (double& value : result.m)
		{
			value = distribution(generator);
		}

		return result;
	}

	Matrix Matrix::diagonal(
		double diagonalValue,
		std::size_t size)
	{
		Matrix result(size, size);

		for (std::size_t i = 0; i < size; ++i)
		{
			result(i, i) = diagonalValue;
		}

		return result;
	}

	std::size_t Matrix::rows() const
	{
		return Rows;
	}

	std::size_t Matrix::columns() const
	{
		return Columns;
	}

	std::size_t Matrix::size() const
	{
		return m.size();
	}

	bool Matrix::empty() const
	{
		return m.empty();
	}

	bool Matrix::isSquare() const
	{
		return Rows == Columns;
	}

	void Matrix::validateSameDimensions(
		const Matrix& other) const
	{
		if (Rows != other.Rows ||
			Columns != other.Columns)
		{
			throw std::invalid_argument(
				"Matrix dimensions must match");
		}
	}

	std::size_t Matrix::checkedElementCount(
		std::size_t rows,
		std::size_t columns)
	{
		if (columns != 0 &&
			rows >
			std::numeric_limits<std::size_t>::max()
			/ columns)
		{
			throw std::length_error(
				"Matrix dimensions are too large");
		}

		return rows * columns;
	}

	Matrix Matrix::add(
		const Matrix& other) const
	{
		validateSameDimensions(other);

		Matrix result(Rows, Columns);

		for (std::size_t i = 0; i < m.size(); ++i)
		{
			result.m[i] = m[i] + other.m[i];
		}

		return result;
	}

	Matrix Matrix::operator+(
		const Matrix& other) const
	{
		return add(other);
	}

	Matrix& Matrix::operator+=(
		const Matrix& other)
	{
		validateSameDimensions(other);

		for (std::size_t i = 0; i < m.size(); ++i)
		{
			m[i] += other.m[i];
		}

		return *this;
	}

	Matrix Matrix::sub(
		const Matrix& other) const
	{
		validateSameDimensions(other);

		Matrix result(Rows, Columns);

		for (std::size_t i = 0; i < m.size(); ++i)
		{
			result.m[i] = m[i] - other.m[i];
		}

		return result;
	}

	Matrix Matrix::operator-(
		const Matrix& other) const
	{
		return sub(other);
	}

	Matrix& Matrix::operator-=(
		const Matrix& other)
	{
		validateSameDimensions(other);

		for (std::size_t i = 0; i < m.size(); ++i)
		{
			m[i] -= other.m[i];
		}

		return *this;
	}

	Matrix Matrix::scale(double scalar) const
	{
		Matrix result = *this;

		for (double& value : result.m)
		{
			value *= scalar;
		}

		return result;
	}

	Matrix Matrix::operator*(double scalar) const
	{
		return scale(scalar);
	}

	Matrix& Matrix::operator*=(double scalar)
	{
		for (double& value : m)
		{
			value *= scalar;
		}

		return *this;
	}

	Matrix Matrix::div(double divisor) const
	{
		if (divisor == 0.0)
		{
			throw std::invalid_argument(
				"Divisor cannot be zero");
		}

		Matrix result = *this;

		for (double& value : result.m)
		{
			value /= divisor;
		}

		return result;
	}

	Matrix Matrix::operator/(double divisor) const
	{
		return div(divisor);
	}

	Matrix& Matrix::operator/=(double divisor)
	{
		if (divisor == 0.0)
		{
			throw std::invalid_argument(
				"Divisor cannot be zero");
		}

		for (double& value : m)
		{
			value /= divisor;
		}

		return *this;
	}

	Matrix Matrix::mult(
		const Matrix& other) const
	{
		if (Columns != other.Rows)
		{
			throw std::invalid_argument(
				"Matrix dimensions do not match for multiplication");
		}

		Matrix result(
			Rows,
			other.Columns);

		for (std::size_t row = 0;
			row < Rows;
			++row)
		{
			for (std::size_t k = 0;
				k < Columns;
				++k)
			{
				const double leftValue =
					(*this)(row, k);

				for (std::size_t column = 0;
					column < other.Columns;
					++column)
				{
					result(row, column) +=
						leftValue * other(k, column);
				}
			}
		}

		return result;
	}

	Matrix Matrix::operator*(
		const Matrix& other) const
	{
		return mult(other);
	}

	Vector3 Matrix::multVector(
		const Vector3& vector) const
	{
		if (Rows != 3 || Columns != 3)
		{
			throw std::invalid_argument(
				"Matrix must be 3x3 to multiply by Vector3");
		}

		return Vector3(
			(*this)(0, 0) * vector.x +
			(*this)(0, 1) * vector.y +
			(*this)(0, 2) * vector.z,

			(*this)(1, 0) * vector.x +
			(*this)(1, 1) * vector.y +
			(*this)(1, 2) * vector.z,

			(*this)(2, 0) * vector.x +
			(*this)(2, 1) * vector.y +
			(*this)(2, 2) * vector.z);
	}

	Vector3 Matrix::operator*(
		const Vector3& vector) const
	{
		return multVector(vector);
	}

	Matrix Matrix::transpose() const
	{
		Matrix result(Columns, Rows);

		for (std::size_t row = 0;
			row < Rows;
			++row)
		{
			for (std::size_t column = 0;
				column < Columns;
				++column)
			{
				result(column, row) =
					(*this)(row, column);
			}
		}

		return result;
	}

	double Matrix::trace() const
	{
		if (!isSquare())
		{
			throw std::domain_error(
				"Trace is only defined for square matrices");
		}

		double result = 0.0;

		for (std::size_t i = 0;
			i < Rows;
			++i)
		{
			result += (*this)(i, i);
		}

		return result;
	}

	double Matrix::determinant() const
	{
		if (!isSquare())
		{
			throw std::domain_error(
				"Determinant is only defined for square matrices");
		}

		const std::size_t n = Rows;

		if (n == 0)
		{
			return 1.0;
		}

		Matrix temp = *this;

		double determinantValue = 1.0;
		bool oddSwapCount = false;

		for (std::size_t column = 0;
			column < n;
			++column)
		{
			std::size_t pivotRow = column;
			double bestRatio = -1.0;

			// Find the best pivot using scaled partial pivoting.
			for (std::size_t row = column;
				row < n;
				++row)
			{
				double rowScale = 0.0;

				for (std::size_t col = column;
					col < n;
					++col)
				{
					rowScale = std::max(
						rowScale,
						std::abs(temp(row, col)));
				}

				if (rowScale == 0.0)
				{
					continue;
				}

				const double ratio =
					std::abs(temp(row, column))
					/ rowScale;

				if (ratio > bestRatio)
				{
					bestRatio = ratio;
					pivotRow = row;
				}
			}

			// No nonzero row remains.
			if (bestRatio < 0.0)
			{
				return 0.0;
			}

			// The selected column has no nonzero pivot.
			if (temp(pivotRow, column) == 0.0)
			{
				return 0.0;
			}

			if (pivotRow != column)
			{
				for (std::size_t col = 0;
					col < n;
					++col)
				{
					std::swap(
						temp(column, col),
						temp(pivotRow, col));
				}

				oddSwapCount = !oddSwapCount;
			}

			const double pivot =
				temp(column, column);

			determinantValue *= pivot;

			for (std::size_t row = column + 1;
				row < n;
				++row)
			{
				const double factor =
					temp(row, column) / pivot;

				temp(row, column) = 0.0;

				for (std::size_t col = column + 1;
					col < n;
					++col)
				{
					temp(row, col) -=
						factor * temp(column, col);
				}
			}
		}

		if (oddSwapCount)
		{
			determinantValue =
				-determinantValue;
		}

		return determinantValue;
	}

	Matrix Matrix::cofactor() const
	{
		if (!isSquare())
		{
			throw std::domain_error(
				"Cofactor is only defined for square matrices");
		}

		const std::size_t n = Rows;

		Matrix result(n, n);

		for (std::size_t row = 0;
			row < n;
			++row)
		{
			for (std::size_t column = 0;
				column < n;
				++column)
			{
				Matrix minorMatrix(
					n - 1,
					n - 1);

				std::size_t minorRow = 0;

				for (std::size_t sourceRow = 0;
					sourceRow < n;
					++sourceRow)
				{
					if (sourceRow == row)
					{
						continue;
					}

					std::size_t minorColumn = 0;

					for (std::size_t sourceColumn = 0;
						sourceColumn < n;
						++sourceColumn)
					{
						if (sourceColumn == column)
						{
							continue;
						}

						minorMatrix(
							minorRow,
							minorColumn) =
							(*this)(
								sourceRow,
								sourceColumn);

						++minorColumn;
					}

					++minorRow;
				}

				const double minorDeterminant =
					minorMatrix.determinant();

				const double sign =
					((row + column) % 2 == 0)
					? 1.0
					: -1.0;

				result(row, column) =
					sign * minorDeterminant;
			}
		}

		return result;
	}

	Matrix Matrix::adjoint() const
	{
		if (!isSquare())
		{
			throw std::domain_error(
				"Adjugate is only defined for square matrices");
		}

		return cofactor().transpose();
	}

	Matrix Matrix::inverse() const
	{
		if (!isSquare())
		{
			throw std::domain_error(
				"Inverse is only defined for square matrices");
		}

		const std::size_t n = Rows;

		if (n == 0)
		{
			return Matrix(0, 0);
		}

		Matrix left = *this;
		Matrix right = identity(n);

		for (std::size_t column = 0;
			column < n;
			++column)
		{
			std::size_t pivotRow = column;
			double bestRatio = -1.0;
			double pivotScale = 0.0;

			// Find the best pivot using scaled partial pivoting.
			for (std::size_t row = column;
				row < n;
				++row)
			{
				double rowScale = 0.0;

				for (std::size_t col = column;
					col < n;
					++col)
				{
					rowScale = std::max(
						rowScale,
						std::abs(left(row, col)));
				}

				if (rowScale == 0.0)
				{
					continue;
				}

				const double ratio =
					std::abs(left(row, column))
					/ rowScale;

				if (ratio > bestRatio)
				{
					bestRatio = ratio;
					pivotRow = row;
					pivotScale = rowScale;
				}
			}

			// No usable pivot exists.
			if (bestRatio < 0.0)
			{
				throw std::domain_error(
					"Matrix is singular and cannot be inverted");
			}

			const double pivotMagnitude =
				std::abs(left(pivotRow, column));

			const double tolerance =
				std::numeric_limits<double>::epsilon()
				* static_cast<double>(n)
				* pivotScale;

			if (pivotMagnitude <= tolerance)
			{
				throw std::domain_error(
					"Matrix is singular or numerically singular");
			}

			// Move the selected pivot row into position.
			if (pivotRow != column)
			{
				for (std::size_t col = 0;
					col < n;
					++col)
				{
					std::swap(
						left(column, col),
						left(pivotRow, col));

					std::swap(
						right(column, col),
						right(pivotRow, col));
				}
			}

			// Normalize the pivot row so the pivot becomes 1.
			const double pivot =
				left(column, column);

			for (std::size_t col = 0;
				col < n;
				++col)
			{
				left(column, col) /= pivot;
				right(column, col) /= pivot;
			}

			// Eliminate the current column from every other row.
			for (std::size_t row = 0;
				row < n;
				++row)
			{
				if (row == column)
				{
					continue;
				}

				const double factor =
					left(row, column);

				if (factor == 0.0)
				{
					continue;
				}

				for (std::size_t col = 0;
					col < n;
					++col)
				{
					left(row, col) -=
						factor * left(column, col);

					right(row, col) -=
						factor * right(column, col);
				}

				// The value should mathematically be zero.
				left(row, column) = 0.0;
			}
		}

		return right;
	}

	bool Matrix::isFinite() const
	{
		for (double value : m)
		{
			if (!std::isfinite(value))
			{
				return false;
			}
		}

		return true;
	}

	bool Matrix::isApprox(
		const Matrix& other,
		double epsilon) const
	{
		if (epsilon <= 0.0)
		{
			throw std::invalid_argument(
				"Epsilon must be positive");
		}

		if (Rows != other.Rows ||
			Columns != other.Columns)
		{
			return false;
		}

		for (std::size_t i = 0;
			i < m.size();
			++i)
		{
			if (!std::isfinite(m[i]) ||
				!std::isfinite(other.m[i]))
			{
				return false;
			}

			const double difference =
				std::abs(m[i] - other.m[i]);

			const double scale =
				std::max({
					1.0,
					std::abs(m[i]),
					std::abs(other.m[i])
					});

			if (difference > epsilon * scale)
			{
				return false;
			}
		}

		return true;
	}

	Matrix Matrix::swapRows(
		std::size_t row1,
		std::size_t row2) const
	{
		if (row1 >= Rows ||
			row2 >= Rows)
		{
			throw std::out_of_range(
				"Matrix row index out of range");
		}

		Matrix result = *this;

		for (std::size_t column = 0;
			column < Columns;
			++column)
		{
			std::swap(
				result(row1, column),
				result(row2, column));
		}

		return result;
	}

	Matrix Matrix::swapColumns(
		std::size_t column1,
		std::size_t column2) const
	{
		if (column1 >= Columns ||
			column2 >= Columns)
		{
			throw std::out_of_range(
				"Matrix column index out of range");
		}

		Matrix result = *this;

		for (std::size_t row = 0;
			row < Rows;
			++row)
		{
			std::swap(
				result(row, column1),
				result(row, column2));
		}

		return result;
	}

	Matrix Matrix::augment(
		const Matrix& other) const
	{
		if (Rows != other.Rows)
		{
			throw std::invalid_argument(
				"Matrices must have the same number of rows to augment");
		}

		Matrix result(
			Rows,
			Columns + other.Columns);

		for (std::size_t row = 0;
			row < Rows;
			++row)
		{
			for (std::size_t column = 0;
				column < Columns;
				++column)
			{
				result(row, column) =
					(*this)(row, column);
			}

			for (std::size_t column = 0;
				column < other.Columns;
				++column)
			{
				result(
					row,
					Columns + column) =
					other(row, column);
			}
		}

		return result;
	}

	Matrix Matrix::subMatrix(
		std::size_t startRow,
		std::size_t startColumn,
		std::size_t rowCount,
		std::size_t columnCount) const
	{
		if (startRow >= Rows ||
			startColumn >= Columns)
		{
			throw std::out_of_range(
				"Submatrix starting position is outside matrix bounds");
		}

		if (rowCount == 0 ||
			columnCount == 0)
		{
			throw std::invalid_argument(
				"Submatrix dimensions must be positive");
		}

		if (rowCount > Rows - startRow ||
			columnCount > Columns - startColumn)
		{
			throw std::out_of_range(
				"Submatrix exceeds matrix bounds");
		}

		Matrix result(
			rowCount,
			columnCount);

		for (std::size_t row = 0;
			row < rowCount;
			++row)
		{
			for (std::size_t column = 0;
				column < columnCount;
				++column)
			{
				result(row, column) =
					(*this)(
						startRow + row,
						startColumn + column);
			}
		}

		return result;
	}

	Matrix toMatrix(const Vector3& vector)
	{
		Matrix result(3, 1);
		result(0, 0) = vector.x;
		result(1, 0) = vector.y;
		result(2, 0) = vector.z;
		return result;
	}

	Matrix operator*(
		double scalar,
		const Matrix& matrix)
	{
		return matrix * scalar;
	}
}