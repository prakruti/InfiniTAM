// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../MiniSlamGraphLib/DualQuaternionHelpers.h"
using namespace ITMLib;

#include <vector> 

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
 //	DQB(scene);

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

// //void DQB(ITMScene<TVoxel,TIndex> *scene, std::map<std::vector<x,y,z>, SE3Pose> mymap){
//   void DQB(ITMScene<TVoxel,TIndex> *scene, std::map<std::vector<float>, ORUtils::SE3Pose> mymap){
// 	int radius = 2;
// 	double distance ;
// 	int k = 2;
// 	std::map<std::vector<float>, ORUtils::SE3Pose> nearest_nbrs;
// 	int hashIdx;
// 	float weight;
	
// 	dq_t prev_dq;
// 	dq_t new_dq ;
// 	typename ITMLib::TIndex::IndexCache cache;
// 	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
// 	ITMHashEntry *hashTable = scene->index.GetEntries();
// 	//for(std::map<Vector3f<x,y,z>, SE3Pose>::iterator it=mymap.begin(); it!=mymap.end(); ++it){
// 	for(auto it=mymap.begin(); it!=mymap.end(); ++it){
// 		//Vector3f<x,y,z> = (it)->first;
// 		Vector3f point;
// 		std::vector<float> coordinate_world = (it)->first;
// 		point.x = coordinate_world.at(0);
// 		point.y= coordinate_world.at(1);
// 		point.z = coordinate_world.at(2);

// 		ORUtils::SE3Pose warp = (it)->second;

// 		// for storing the dqb for all the nodes
		
// 		// sanity check if block coords are not non existent
// 		for(int i=point.x-radius; i<=point.x+radius; i++){
// 			for(int j=point.y-radius; j<=point.y+radius; j++){
// 				for(int k=point.z-radius; k<=point.z+radius; k++){

// 					// for each voxel in this surrounding, check k nearest nodes from the map
// 					Vector3f point = Vector3f(i,j,k);
				    
// 					Vector3s blockPos = TO_SHORT_FLOOR3(point);

// 					hashIdx = hashIndex(blockPos); 

// 					// find the weight for the neighbouring points
// 					distance = ((point.x,point.y,point.z)-(i,j,k)).norm();

// 					weight = exp(-distnace^2) ;


                    
// 					//readVoxel(localVBA, hashTable, Vector3i, vmIndex,cache);
// 					TVoxel result = readVoxel(localVBA, hashTable, point, vmIndex, cache);
					
// 					if ( !result.dg_se3)

// 					result.dg_se3 = dq_dquat_to_se3(weight*dq_se3_to_dquat(warp));

// 					else 

// 					prev_dq = dq_se3_to_dquat(hashIdx->dq_se3);
// 					new_dq = dq_op_add(prev_dq,weight*dq_se3_to_dquat(warp));
// 					result.dg_se3 = dq_dquat_to_se3(new_dq);

// 					//dqb neighbour = distnace*dq_SE3toQuad(warp);
// 					//temp_dqb.push_back(neighbour);
					

// 					// check all nodes, find closest // 
// 					// for(std::map<vector<x,y,z>, SE3Pose>::iterator nbr_it=mymap.begin(); nbr_it!=mymap.end(); ++nbr_it){
// 					// 	vector<x,y,z> = (nbr_it)->first;
// 					// 	SE3 warp = (nbr_it)->second;

// 					// 	// find distance
// 					// 	distance = ((x,y,z)-(i,j,k)).norm();



// 					// 	// find k closest nodes


// 					// 	// for each of these closest nodes, update voxel warp value
// 					// 	// do a scalar multiplication to the warp 
// 					// 	voxel(i,j,k).warp_value += dq_scalar_multiplication(distance*dq_SE3toQuad(warp));
// 					// 	// normaalizing the warp value 
// 					// 	voxel(i,j,k).warp_value = dq_op_norm2(warp_value) ;



// 					}



// 				}
// 			}
// 		}

// 	}

	// make all blended quaternions unit (normalize)
	// convert them back into SE3









