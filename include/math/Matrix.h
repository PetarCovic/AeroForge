#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

#include <math/Vector3.h>

namespace aeroforge::math
{
	struct Matrix
	{
		// Construction
		Matrix(std::size_t rows, std::size_t columns);

		Matrix(
			std::size_t rows,
			std::size_t columns,
			double initialValue);

		// Set every element individually in row-major order.
		// Example:
		// Matrix(2, 3, {1, 2, 3, 4, 5, 6});
		Matrix(
			std::size_t rows,
			std::size_t columns,
			std::initializer_list<double> values);

		// Convenient matrix-literal construction.
		// Example:
		//
		// Matrix m{
		//     {1, 2, 3},
		//     {4, 5, 6}
		// };
		Matrix(
			std::initializer_list<
			std::initializer_list<double>> values);

		// Value access
		Matrix& setValue(
			std::size_t row,
			std::size_t column,
			double value);

		double getValue(
			std::size_t row,
			std::size_t column) const;

		// Set every matrix entry individually.
		Matrix& setValues(
			std::initializer_list<double> values);

		Matrix& setValues(
			std::initializer_list<
			std::initializer_list<double>> values);

		// Set every entry to the same value.
		Matrix& fill(double value);

		double& operator()(
			std::size_t row,
			std::size_t column);

		const double& operator()(
			std::size_t row,
			std::size_t column) const;

		// Factories
		static Matrix identity(std::size_t size);

		static Matrix zeros(
			std::size_t rows,
			std::size_t columns);

		static Matrix ones(
			std::size_t rows,
			std::size_t columns);

		static Matrix random(
			std::size_t rows,
			std::size_t columns);

		// Useful for deterministic AeroForge tests/simulations.
		static Matrix random(
			std::size_t rows,
			std::size_t columns,
			std::uint64_t seed);

		static Matrix diagonal(
			double diagonalValue,
			std::size_t size);

		// Dimensions
		std::size_t rows() const;
		std::size_t columns() const;
		std::size_t size() const;
		bool empty() const;
		bool isSquare() const;

		// Addition
		Matrix add(const Matrix& other) const;
		Matrix operator+(const Matrix& other) const;
		Matrix& operator+=(const Matrix& other);

		// Subtraction
		Matrix sub(const Matrix& other) const;
		Matrix operator-(const Matrix& other) const;
		Matrix& operator-=(const Matrix& other);

		// Scalar multiplication
		Matrix scale(double scalar) const;
		Matrix operator*(double scalar) const;
		Matrix& operator*=(double scalar);

		// Scalar division
		Matrix div(double divisor) const;
		Matrix operator/(double divisor) const;
		Matrix& operator/=(double divisor);

		// Matrix multiplication
		Matrix mult(const Matrix& other) const;
		Matrix operator*(const Matrix& other) const;

		// Matrix × Vector3
		Vector3 multVector(const Vector3& vector) const;
		Vector3 operator*(const Vector3& vector) const;

		// Matrix properties / operations
		Matrix transpose() const;
		double trace() const;
		double determinant() const;

		// Keep these even though inverse() will not use them.
		Matrix cofactor() const;
		Matrix adjoint() const;

		Matrix inverse() const;

		bool isFinite() const;
		bool isApprox(
			const Matrix& other,
			double epsilon) const;

		Matrix swapRows(
			std::size_t row1,
			std::size_t row2) const;

		Matrix swapColumns(
			std::size_t column1,
			std::size_t column2) const;

		Matrix augment(const Matrix& other) const;

		Matrix subMatrix(
			std::size_t startRow,
			std::size_t startColumn,
			std::size_t rowCount,
			std::size_t columnCount) const;

		bool isEqual(const Matrix& other) const;

		bool isSymmetric(double tolerance = 1e-12) const;

		bool isPositiveDefinite(double tolerance = 1e-12) const;

	private:
		std::size_t Rows;
		std::size_t Columns;

		// Contiguous row-major storage.
		std::vector<double> m;

		std::size_t index(
			std::size_t row,
			std::size_t column) const;

		void validateSameDimensions(
			const Matrix& other) const;

		static std::size_t checkedElementCount(
			std::size_t rows,
			std::size_t columns);
	};

	Matrix toMatrix(const Vector3& vector);

	Matrix operator*(
		double scalar,
		const Matrix& matrix);
}