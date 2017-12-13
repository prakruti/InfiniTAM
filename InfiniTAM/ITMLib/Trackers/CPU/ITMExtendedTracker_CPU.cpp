// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_CPU.h"
#include "../Shared/ITMExtendedTracker_Shared.h"
#include <iostream> 

using namespace ITMLib;

ITMExtendedTracker_CPU::ITMExtendedTracker_CPU(Vector2i imgSize_d,
											   Vector2i imgSize_rgb,
											   bool useDepth,
											   bool useColour,
											   float colourWeight,
											   TrackerIterationType *trackingRegime,
											   int noHierarchyLevels,
											   float terminationThreshold,
											   float failureDetectorThreshold,
											   float viewFrustum_min,
											   float viewFrustum_max,
											   float minColourGradient,
											   float tukeyCutOff,
											   int framesToSkip,
											   int framesToWeight,
											   const ITMLowLevelEngine *lowLevelEngine)
	: ITMExtendedTracker(imgSize_d,
						 imgSize_rgb,
						 useDepth,
						 useColour,
						 colourWeight,
						 trackingRegime,
						 noHierarchyLevels,
						 terminationThreshold,
						 failureDetectorThreshold,
						 viewFrustum_min,
						 viewFrustum_max,
						 minColourGradient,
						 tukeyCutOff,
						 framesToSkip,
						 framesToWeight,
						 lowLevelEngine,
						 MEMORYDEVICE_CPU)
{ }

ITMExtendedTracker_CPU::~ITMExtendedTracker_CPU(void) { }

int ITMExtendedTracker_CPU::ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap = sceneHierarchyLevel_Depth->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel_Depth->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel_Depth->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel_Depth->pointsMap->noDims;

	float *depth = viewHierarchyLevel_Depth->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = viewHierarchyLevel_Depth->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel_Depth->depth->noDims;

	if (currentIterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (currentIterationType == TRACKER_ITERATION_ROTATION)
						   || (currentIterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint;

		float depthWeight;

		if (framesProcessed < 100)
		{
			switch (currentIterationType)
			{
			case TRACKER_ITERATION_ROTATION:
				isValidPoint = computePerPointGH_exDepth<true, true, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_TRANSLATION:
				isValidPoint = computePerPointGH_exDepth<true, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_BOTH:
				isValidPoint = computePerPointGH_exDepth<false, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			default:
				isValidPoint = false;
				break;
			}
		}
		else
		{
			switch (currentIterationType)
			{
			case TRACKER_ITERATION_ROTATION:
				isValidPoint = computePerPointGH_exDepth<true, true, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_TRANSLATION:
				isValidPoint = computePerPointGH_exDepth<true, false, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_BOTH:
				isValidPoint = computePerPointGH_exDepth<false, false, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			default:
				isValidPoint = false;
				break;
			}
		}

		if (isValidPoint)
		{
			noValidPoints++;
			sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	// Copy the lower triangular part of the matrix.
	for (int r = 0, counter = 0; r < noPara; r++)
		for (int c = 0; c <= r; c++, counter++)
			hessian[r + c * 6] = sumHessian[counter];

	// Transpose to fill the upper triangle.
	for (int r = 0; r < noPara; ++r)
		for (int c = r + 1; c < noPara; c++)
			hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));

	f = sumF;

	return noValidPoints;
}

int ITMExtendedTracker_CPU::ComputeGandH_Depth_patch(float &f, float *nabla, float *hessian, Matrix4f approxInvPose, Matrix4f approxInvWarp, int start_x, int start_y, int patch_size_x, int patch_size_y, ITMTrackingState *trackingState)
{
	Vector4f *pointsMap = sceneHierarchyLevel_Depth->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel_Depth->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel_Depth->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel_Depth->pointsMap->noDims;

	float *depth = viewHierarchyLevel_Depth->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = viewHierarchyLevel_Depth->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel_Depth->depth->noDims;


	if (currentIterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (currentIterationType == TRACKER_ITERATION_ROTATION)
						   || (currentIterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	//How can I access the warp of a voxel when I don't have access to the map? 

	Matrix4f sceneWarp;
	sceneWarp.setIdentity();

	for (int y = start_y; y < start_y+ patch_size_y; y++) for (int x = start_x; x < start_x + patch_size_x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint;

		float depthWeight;

		Vector4f tmp3Dpoint;
		float Z = depth[x + y * viewImageSize.x];
		tmp3Dpoint.x = Z * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
		tmp3Dpoint.y = Z * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
		tmp3Dpoint.z = Z;
		tmp3Dpoint.w = 1.0f;


		tmp3Dpoint = approxInvWarp * approxInvPose * tmp3Dpoint;
		tmp3Dpoint.w = 1.0f;

		//tmp3Dpoint is the point in world frame at t=0

		//Interpolate the warp at this point using the warps at node locations
		std::map<Vector3f, ORUtils::SE3Pose *>::iterator it;
		float min_dist = std::numeric_limits<float>::max();
		float threshold = 10;

		for(auto it=trackingState->prev_warp_field_XYZ.begin(); it!=trackingState->prev_warp_field_XYZ.end(); it++)
		{
			Vector3f node_location = it->first;
			Vector3f ptDiff;
			ptDiff.x = tmp3Dpoint.x - node_location.x;
			ptDiff.y = tmp3Dpoint.y - node_location.y;
			ptDiff.z = tmp3Dpoint.z - node_location.z;

			float dist = ptDiff.x*ptDiff.x + ptDiff.y*ptDiff.y + ptDiff.z*ptDiff.z;

			if((dist< min_dist) && min_dist < threshold)
			{
				sceneWarp = (it->second)->GetM();
			}
		}
		
		// sceneWarp = getInterpolatedWarp(tmp3Dpoint, trackingState->prev_warp_field);
		// std::cout << "Scene Warp\n";
		// std::cout << sceneWarp << std::endl;

		if (framesProcessed < 100)
		{
			switch (currentIterationType)
			{
			case TRACKER_ITERATION_ROTATION:
				isValidPoint = computePerPointGH_exDepth_patch<true, true, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, approxInvWarp, scenePose, sceneWarp, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_TRANSLATION:
				isValidPoint = computePerPointGH_exDepth_patch<true, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, approxInvWarp, scenePose, sceneWarp, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_BOTH:
				isValidPoint = computePerPointGH_exDepth_patch<false, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, approxInvWarp, scenePose, sceneWarp, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			default:
				isValidPoint = false;
				break;
			}
		}
		else
		{
			switch (currentIterationType)
			{
			case TRACKER_ITERATION_ROTATION:
				isValidPoint = computePerPointGH_exDepth_patch<true, true, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, approxInvWarp, scenePose, sceneWarp, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_TRANSLATION:
				isValidPoint = computePerPointGH_exDepth_patch<true, false, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, approxInvWarp, scenePose, sceneWarp, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			case TRACKER_ITERATION_BOTH:
				isValidPoint = computePerPointGH_exDepth_patch<false, false, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
					viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, approxInvWarp, scenePose, sceneWarp, pointsMap, normalsMap, spaceThresh[currentLevelId],
					viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
				break;
			default:
				isValidPoint = false;
				break;
			}
		}

		if (isValidPoint)
		{
			noValidPoints++;
			sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	// Copy the lower triangular part of the matrix.
	for (int r = 0, counter = 0; r < noPara; r++)
		for (int c = 0; c <= r; c++, counter++)
			hessian[r + c * 6] = sumHessian[counter];

	// Transpose to fill the upper triangle.
	for (int r = 0; r < noPara; ++r)
		for (int c = r + 1; c < noPara; c++)
			hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));

	f = sumF;

	return noValidPoints;
}

int ITMExtendedTracker_CPU::ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	const Vector2i viewImageSize_depth = viewHierarchyLevel_Depth->depth->noDims;
	const Vector2i viewImageSize_rgb = viewHierarchyLevel_Intensity->intensity_prev->noDims;

	const Vector4f *points_curr = reprojectedPointsLevel->data->GetData(MEMORYDEVICE_CPU);
	const float *intensities_prev = viewHierarchyLevel_Intensity->intensity_prev->GetData(MEMORYDEVICE_CPU);
	const float *intensities_current = projectedIntensityLevel->data->GetData(MEMORYDEVICE_CPU);
	const Vector2f *gradients = viewHierarchyLevel_Intensity->gradients->GetData(MEMORYDEVICE_CPU);

	Vector4f projParams_rgb = viewHierarchyLevel_Intensity->intrinsics;
	Vector4f projParams_depth = viewHierarchyLevel_Depth->intrinsics;

	if (currentIterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (currentIterationType == TRACKER_ITERATION_ROTATION)
						   || (currentIterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF;
	int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	for (int y = 0; y < viewImageSize_depth.y; y++) for (int x = 0; x < viewImageSize_depth.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint = false;

		switch (currentIterationType)
		{
		case TRACKER_ITERATION_ROTATION:
			isValidPoint = computePerPointGH_exRGB_inv_Ab<true, true>(
					localF,
					localNabla,
					localHessian,
					x,
					y,
					points_curr,
					intensities_current,
					intensities_prev,
					gradients,
					viewImageSize_depth,
					viewImageSize_rgb,
					projParams_depth,
					projParams_rgb,
					approxInvPose,
					depthToRGBTransform * scenePose,
					colourThresh[currentLevelId],
					minColourGradient,
					viewFrustum_min,
					viewFrustum_max,
					tukeyCutOff
					);
			break;
		case TRACKER_ITERATION_TRANSLATION:
			isValidPoint = computePerPointGH_exRGB_inv_Ab<true, false>(
					localF,
					localNabla,
					localHessian,
					x,
					y,
					points_curr,
					intensities_current,
					intensities_prev,
					gradients,
					viewImageSize_depth,
					viewImageSize_rgb,
					projParams_depth,
					projParams_rgb,
					approxInvPose,
					depthToRGBTransform * scenePose,
					colourThresh[currentLevelId],
					minColourGradient,
					viewFrustum_min,
					viewFrustum_max,
					tukeyCutOff
					);
			break;
		case TRACKER_ITERATION_BOTH:
			isValidPoint = computePerPointGH_exRGB_inv_Ab<false, false>(
					localF,
					localNabla,
					localHessian,
					x,
					y,
					points_curr,
					intensities_current,
					intensities_prev,
					gradients,
					viewImageSize_depth,
					viewImageSize_rgb,
					projParams_depth,
					projParams_rgb,
					approxInvPose,
					depthToRGBTransform * scenePose,
					colourThresh[currentLevelId],
					minColourGradient,
					viewFrustum_min,
					viewFrustum_max,
					tukeyCutOff
					);
			break;
		default:
			isValidPoint = false;
			break;
		}

		if (isValidPoint)
		{
			noValidPoints++;
			sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	// Copy the lower triangular part of the matrix.
	for (int r = 0, counter = 0; r < noPara; r++)
		for (int c = 0; c <= r; c++, counter++)
			hessian[r + c * 6] = sumHessian[counter];

	// Transpose to fill the upper triangle.
	for (int r = 0; r < noPara; ++r)
		for (int c = r + 1; c < noPara; c++)
			hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));

	f = sumF;

	return noValidPoints;
}

void ITMExtendedTracker_CPU::ProjectCurrentIntensityFrame(ITMFloat4Image *points_out,
														  ITMFloatImage *intensity_out,
														  const ITMFloatImage *intensity_in,
														  const ITMFloatImage *depth_in,
														  const Vector4f &intrinsics_depth,
														  const Vector4f &intrinsics_rgb,
														  const Matrix4f &scenePose)
{
	const Vector2i imageSize_rgb = intensity_in->noDims;
	const Vector2i imageSize_depth = depth_in->noDims; // Also the size of the projected image

	points_out->ChangeDims(imageSize_depth); // Actual reallocation should happen only once per run.
	intensity_out->ChangeDims(imageSize_depth); // Actual reallocation should happen only once per run.

	const float *depths = depth_in->GetData(MEMORYDEVICE_CPU);
	const float *intensityIn = intensity_in->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsOut = points_out->GetData(MEMORYDEVICE_CPU);
	float *intensityOut = intensity_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imageSize_depth.y; y++) for (int x = 0; x < imageSize_depth.x; x++)
		projectPoint_exRGB(x, y, pointsOut, intensityOut, intensityIn, depths, imageSize_rgb, imageSize_depth, intrinsics_rgb, intrinsics_depth, scenePose);
}
