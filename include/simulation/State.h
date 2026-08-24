#pragma once

#include <vector>
#include <cstddef>
#include <initializer_list>

namespace aeroforge::simulation
{
	class State
	{
	private:
		std::vector<double> values;

	public:
		State();
		explicit State(std::size_t size);
		State(std::initializer_list<double> values);

		std::size_t size() const;
		bool empty() const;

		double& operator[](std::size_t index);
		const double& operator[](std::size_t index) const;

		State operator+(const State& other) const;
		State& operator+=(const State& other);

		State operator-(const State& other) const;
		State& operator-=(const State& other);

		State operator*(double scalar) const;
		State& operator*=(double scalar);

		State operator/(double scalar) const;
		State& operator/=(double scalar);

		bool isFinite() const;

		bool isApprox(const State& other, double epsilon) const;
	};

	State operator*(double scalar, const State& state);
}