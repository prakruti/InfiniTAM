// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../Objects/RenderStates/ITMRenderState_VH.h"
#include "../MiniSlamGraphLib/DualQuaternionHelpers.h"
using namespace ITMLib;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::ITMDenseMapper(const ITMLibSettings *settings)
{
	sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel,TIndex>(settings->deviceType);
	swappingEngine = settings->swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxel,TIndex>(settings->deviceType) : NULL;

	swappingMode = settings->swappingMode;
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	delete swappingEngine;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ResetScene(ITMScene<TVoxel,TIndex> *scene) const
{
	sceneRecoEngine->ResetScene(scene);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState)
{

	// Dual Quaternion blending
	DQB()

	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState);

	if (swappingEngine != NULL) {
		// swapping: CPU -> GPU
		if (swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED) swappingEngine->IntegrateGlobalIntoLocal(scene, renderState);

		// swapping: GPU -> CPU
		switch (swappingMode)
		{
		case ITMLibSettings::SWAPPINGMODE_ENABLED:
			swappingEngine->SaveToGlobalMemory(scene, renderState);
			break;
		case ITMLibSettings::SWAPPINGMODE_DELETE:
			swappingEngine->CleanLocalMemory(scene, renderState);
			break;
		case ITMLibSettings::SWAPPINGMODE_DISABLED:
			break;
		} 
	}
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState, bool resetVisibleList)
{
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}

DQB(){
	// Map<vector<x,y,z>, SE3Pose>
	int radius = 2;
	double distance ;
	int k = 2;
	std::map<vector<x,y,z>, SE3Pose> nearest_nbrs;

	for(std::map<vector<x,y,z>, SE3Pose>::iterator it=mymap.begin(); it!=mymap.end(); ++it){
		vector<x,y,z> = (it)->first;
		SE3 warp = (it)->second;
		vector<dqb> temp_dqb;
		// sanity check if block coords are not non existent
		for(int i=x-radius; i<=x+radius; i++){
			for(int j=y-radius; j<=y+radius; j++){
				for(int k=z-radius; k<=z+radius; k++){

					// for each voxel in this surrounding, check k nearest nodes from the map
					point = (i,j,k);
					blockPos = TO_SHORT_FLOOR3(point);
					hashIdx = hashIndex(blockPos);

					distance = ((x,y,z)-(i,j,k)).norm();

					dqb neighbour = distnace*dq_SE3toQuad(warp);
					temp_dqb.push_back(neighbour);
					

					// check all nodes, find closest
					for(std::map<vector<x,y,z>, SE3Pose>::iterator nbr_it=mymap.begin(); nbr_it!=mymap.end(); ++nbr_it){
						vector<x,y,z> = (nbr_it)->first;
						SE3 warp = (nbr_it)->second;

						// find distance
						distance = ((x,y,z)-(i,j,k)).norm();

						// find k closest nodes


						// for each of these closest nodes, update voxel warp value
						voxel(i,j,k) += distance*warp;



					}



				}
			}
		}

	}

	// make all blended quaternions unit (normalize)
	// convert them back into SE3

}







