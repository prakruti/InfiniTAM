// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMForceFailTracker.h"

namespace ITMLib
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ITMForceFailTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;
}

void ITMForceFailTracker::EstimateWarpField(ITMTrackingState *trackingState, const ITMView *view)
{
	return;
}

bool ITMForceFailTracker::requiresColourRendering() const
{
	return false;
}

bool ITMForceFailTracker::requiresDepthReliability() const
{
	return false;
}

bool ITMForceFailTracker::requiresPointCloudRendering() const
{
	return false;
}

}
