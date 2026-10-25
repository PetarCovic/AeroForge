#include <reproducibility/SeedManager.h>

namespace aeroforge::reproducibility
{
	namespace
	{
		constexpr std::uint64_t TRIAL_CONSTANT =
			0xD1B54A32D192ED03ULL;

		constexpr std::uint64_t STREAM_CONSTANT =
			0x94D049BB133111EBULL;

		std::uint64_t mix64(std::uint64_t value)
		{
			value += 0x9E3779B97F4A7C15;

			value = (value ^ (value >> 30))
				* 0xBF58476D1CE4E5B9;

			value = (value ^ (value >> 27))
				* 0x94D049BB133111EB;

			value = value ^ (value >> 31);

			return value;
		}
	}

	SeedManager::SeedManager(std::uint32_t baseSeed)
		: baseSeed(baseSeed)
	{

	}

	std::uint32_t SeedManager::deriveSeed(std::size_t trialIndex, std::uint32_t streamId) const
	{
		std::uint64_t mixed = mix64(static_cast<std::uint64_t>(baseSeed));

		mixed ^= mix64(static_cast<std::uint64_t>(trialIndex)+TRIAL_CONSTANT);

		mixed ^= mix64(static_cast<std::uint64_t>(streamId)+STREAM_CONSTANT);

		mixed = mix64(mixed);

		std::uint32_t seed = static_cast<std::uint32_t>(mixed ^ (mixed >> 32));

		return seed;
	}

	std::uint32_t SeedManager::getBaseSeed() const
	{
		return baseSeed;
	}
}