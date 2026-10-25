#pragma once

#include <cstddef>
#include <cstdint>

namespace aeroforge::reproducibility
{
	class SeedManager
	{
	public:
		SeedManager(std::uint32_t baseSeed);

		std::uint32_t deriveSeed(std::size_t trialIndex, std::uint32_t streamId) const;

		std::uint32_t getBaseSeed() const;

	private:
		std::uint32_t baseSeed;
	};
}