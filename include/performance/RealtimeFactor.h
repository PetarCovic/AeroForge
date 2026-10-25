#pragma once

namespace aeroforge::performance
{
	class RealtimeFactor
	{
	public:
		static double compute(double simulatedTimeS, double wallClockTimeS);
	};
}