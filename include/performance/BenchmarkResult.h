#pragma once

#include <string>
#include <cstddef>

namespace aeroforge::performance
{
	struct BenchmarkResult
	{
		std::string name;
		std::size_t iterations;

		double totalTimeS;
		double meanTimeS;
		double minTimeS;
		double maxTimeS;

		double operationsPerSecond;
	};
}