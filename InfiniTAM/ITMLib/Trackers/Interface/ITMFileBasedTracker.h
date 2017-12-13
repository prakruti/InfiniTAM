// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"

namespace ITMLib
{
	/**
	 * \brief Tracker that reads precomputed poses from text files.
	 */
	class ITMFileBasedTracker : public ITMTracker
	{
	private:
		std::string poseMask;
		size_t frameCount;

	public:
		bool CanKeepTracking() const;
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);
		void EstimateWarpField(ITMTrackingState *trackingState, const ITMView *view);
		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return false; }

		explicit ITMFileBasedTracker(const std::string &poseMask);

	private:
		std::string GetCurrentFilename() const;
	};
}
