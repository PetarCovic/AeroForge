#include <simulation/State.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aeroforge::simulation
{
	State::State() = default;

	State::State(std::size_t size)
		: values(size)
	{
	}

	State::State(
		std::initializer_list<double> values)
		: values(values)
	{
	}

	std::size_t State::size() const
	{
		return this->values.size();
	}

	bool State::empty() const
	{
		return this->values.empty();
	}

	double& State::operator[](
		std::size_t index)
	{
		return this->values.at(index);
	}

	const double& State::operator[](
		std::size_t index) const
	{
		return this->values.at(index);
	}

	State State::operator+(
		const State& other) const
	{
		State result = *this;

		result += other;

		return result;
	}

	State& State::operator+=(
		const State& other)
	{
		if (this->size() != other.size())
		{
			throw std::invalid_argument(
				"State dimensions must match");
		}

		for (std::size_t i = 0;
			i < this->size();
			++i)
		{
			this->values[i] +=
				other.values[i];
		}

		return *this;
	}

	State State::operator-(
		const State& other) const
	{
		State result = *this;

		result -= other;

		return result;
	}

	State& State::operator-=(
		const State& other)
	{
		if (this->size() != other.size())
		{
			throw std::invalid_argument(
				"State dimensions must match");
		}

		for (std::size_t i = 0;
			i < this->size();
			++i)
		{
			this->values[i] -=
				other.values[i];
		}

		return *this;
	}

	State State::operator*(
		double scalar) const
	{
		State result = *this;

		result *= scalar;

		return result;
	}

	State& State::operator*=(
		double scalar)
	{
		for (double& value : this->values)
		{
			value *= scalar;
		}

		return *this;
	}

	State State::operator/(
		double scalar) const
	{
		State result = *this;

		result /= scalar;

		return result;
	}

	State& State::operator/=(
		double scalar)
	{
		if (scalar == 0.0)
		{
			throw std::invalid_argument(
				"State divisor cannot be zero");
		}

		for (double& value : this->values)
		{
			value /= scalar;
		}

		return *this;
	}

	bool State::isFinite() const
	{
		for (double value : this->values)
		{
			if (!std::isfinite(value))
			{
				return false;
			}
		}

		return true;
	}

	bool State::isApprox(
		const State& other,
		double epsilon) const
	{
		if (epsilon <= 0.0)
		{
			throw std::invalid_argument(
				"Epsilon must be greater than zero");
		}

		if (this->size() != other.size())
		{
			return false;
		}

		if (!this->isFinite() ||
			!other.isFinite())
		{
			return false;
		}

		for (std::size_t i = 0;
			i < this->size();
			++i)
		{
			const double a =
				this->values[i];

			const double b =
				other.values[i];

			const double difference =
				std::abs(a - b);

			const double scale =
				std::max({
					1.0,
					std::abs(a),
					std::abs(b)
					});

			if (difference >
				epsilon * scale)
			{
				return false;
			}
		}

		return true;
	}

	State operator*(
		double scalar,
		const State& state)
	{
		return state * scalar;
	}
}