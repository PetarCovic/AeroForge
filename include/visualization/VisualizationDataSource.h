#pragma once

#include <visualization/VisualizationSnapshot.h>

#include <cstddef>

namespace aeroforge::visualization
{
	class VisualizationDataSource
	{
	public:
		virtual ~VisualizationDataSource()=default;

		virtual bool empty() const=0;

		virtual std::size_t size() const=0;

		virtual double getStartTimeS() const=0;

		virtual double getEndTimeS() const=0;

		virtual VisualizationSnapshot sampleAt(double simulationTimeS) const=0;
	};
}