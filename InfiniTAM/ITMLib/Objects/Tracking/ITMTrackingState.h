// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/SE3Pose.h"
#include "../Misc/ITMPointCloud.h"
#include "../../../MiniSlamGraphLib/DualQuaternionHelpers.h"
#include <vector> 
#include <math.h>
#include <stdio.h>
#include <map>

namespace ITMLib
{
	/** \brief
	    Stores some internal variables about the current tracking
	    state, most importantly the camera pose
	*/
	class ITMTrackingState
	{
	public:
		/** @brief
		    Excerpt of the scene used by the tracker to align
		    a new frame.

		    This is usually the main result generated by the
		    raycasting operation in a ITMLib::Engine::ITMSceneReconstructionEngine.
		*/
		ITMPointCloud *pointCloud;

		/// The pose used to generate the point cloud.
		ORUtils::SE3Pose *pose_pointCloud;

		/// Frames processed from start of tracking
		/// Used as weight in the extended tracker 
		int framesProcessed;

		int age_pointCloud;

		/// Current pose of the depth camera.
		//TODO (@pgogia) : Treat this as the 
		ORUtils::SE3Pose *pose_d;

		//TODO (@pgogia) : Add the warp field term that would hold a vector or SE3Pose per node 
		//Possibly as a map<NodeLocation, SE3Pose> 
		//std::map<Vector3f, ORUtils::SE3Pose> warp_field;

		/// Tracking quality: 1.0: success, 0.0: failure
		enum TrackingResult
		{
			TRACKING_GOOD = 2,
			TRACKING_POOR = 1,
			TRACKING_FAILED = 0
		} trackerResult;

		bool TrackerFarFromPointCloud(void) const
		{
			// if no point cloud exists, yet
			if (age_pointCloud < 0) return true;
			// if the point cloud is older than n frames
			if (age_pointCloud > 5) return true;

			Vector3f cameraCenter_pc = -1.0f * (pose_pointCloud->GetR().t() * pose_pointCloud->GetT());
			Vector3f cameraCenter_live = -1.0f * (pose_d->GetR().t() * pose_d->GetT());

			Vector3f diff3 = cameraCenter_pc - cameraCenter_live;

			float diff = diff3.x * diff3.x + diff3.y * diff3.y + diff3.z * diff3.z;

			// if the camera center has moved by more than a threshold
			if (diff > 0.0005f) return true;

			return false;
		}

		ITMTrackingState(Vector2i imgSize, MemoryDeviceType memoryType)
		: pointCloud(new ITMPointCloud(imgSize, memoryType)),
			pose_pointCloud(new ORUtils::SE3Pose),
			pose_d(new ORUtils::SE3Pose)
		{
			Reset();
		}

		
		 ORUtils::SE3Pose Interpolate3DWarp(std::map<Vector3f, ORUtils::SE3Pose> map_nodes, Vector3f world_coord)
		{
		// min_dist : gives the set of three distances which are neares nodes 
		//poses: gives the set of  poses for the three nearest nodes
		// temp : gives the value of vector 3f for each node 
		// temp_se3 : gives the value of se3 for each node 
		// diffence : gives the difference for node and world point 

			std::vector<float> min_dist ;
			std::vector<ORUtils::SE3Pose> poses;
			Vector3f temp;
			ORUtils::SE3Pose temp_se3;
			float difference;
			ORUtils::SE3Pose final_warp;

			int i=0;
			// run the for loop for all the nodes 
			for(auto it=map_nodes.begin(); it!=map_nodes.end(); ++it)
			//for (size_t i = 0; i <= map_nodes.size();  i++)
			{      
				//auto it = map_nodes[i];
				temp = it->first;
				difference = pow(world_coord.x - temp.x,2) + pow(world_coord.y - temp.y,2) + pow(world_coord.z - temp.z,2) ;
				temp_se3 = it->second; 
				

				// start the min_dist and the poses
				if ( i ==0)
				{
					poses.push_back(temp_se3);
					min_dist.push_back(difference);
				}

				if(i ==1 )
				{
					if (difference < min_dist[0])
					{
						min_dist[1] = min_dist[0];
						poses[1] = poses[0]; 
						min_dist[0] = difference;
						poses[0] = temp_se3;
					}
					else 
					{
					poses.push_back(temp_se3);
					min_dist.push_back(difference);

					}
				}
				// chekc for the third case 

				if ( i ==2 )
				{
					if ( difference < min_dist[0])
					{

						min_dist[2] = min_dist[1];
						poses[2] = poses[1]; 
						min_dist[1] = min_dist[0];
						poses[1] = poses[0]; 
						min_dist[0] = difference;
						poses[0] = temp_se3;
					}
				
					else if ( difference < min_dist[1] &&  difference > min_dist[0])
					{

						min_dist[2] = min_dist[1];
						poses[2] = poses[1]; 
						min_dist[1] = difference;
						poses[1] = temp_se3; 
		
					}

				}

				// after population the first 3 distances and the node se 3 values , keep the ordered array  
			    if ( i >2)
			    {
				    // check if the new difference is smallest in the array 
					if ( difference < min_dist[0])
					{
						min_dist[2] = min_dist[1];
						poses[2] = poses[1]; 
						min_dist[1] = min_dist[0];
						poses[1] = poses[0]; 
						min_dist[0] = difference;
						poses[0] = temp_se3;
					}
					// chekc if the new diffenrce is greater than smalles but larger than the second biggest.
					else if ( difference < min_dist[1] &&  difference > min_dist[0])
					{

						min_dist[2] = min_dist[1];
						poses[2] = poses[1]; 
						min_dist[1] = difference;
						poses[1] = temp_se3; 
		
					}
					// chekc if the new diffenrece is smaller than the largers but larger than the second largest. 
					else if ( difference < min_dist[2] && difference > min_dist[1])
					{

						min_dist[2] = difference;
						poses[2] = temp_se3;

					}

				}
				

				std::vector<double> weight;
				//std::vector<dq_t*> dual_quat;


				weight[0] = exp(pow(min_dist[0],2)) ;
				weight[1] = exp(pow(min_dist[1],2)) ;
				weight[2] = exp(pow(min_dist[2],2)) ;
				
				dq_t dual_quat_zero;
				dq_t dual_quat_one;
				dq_t dual_quat_two;

				dq_se3_to_dquat(dual_quat_zero , poses[0]);
				dq_se3_to_dquat(dual_quat_one , poses[0]);
				dq_se3_to_dquat(dual_quat_two , poses[0]);

				dq_t dual_quat_addone ;
				dq_t dual_quat_average;

				for(int k = 0; i < 8 ;  i++) dual_quat_zero[i] *= weight[0];
				for(int k = 0; i < 8 ;  i++) dual_quat_one[i] *= weight[1];
				for(int k = 0; i < 8 ;  i++) dual_quat_two[i] *= weight[2];
				dq_op_add(dual_quat_addone ,  dual_quat_zero, dual_quat_one );
			    dq_op_add(dual_quat_average,  dual_quat_addone, dual_quat_two );
				//dq_t average = weight[0]*dual_quat[0] + weight[1]*dual_quat[1] + weight[2]*dual_quat[2];
				
				//mulitpilication 
				//addition
				//normalization

				final_warp = dq_dquat_to_se3(dual_quat_average);

				i++;
			} 
			return final_warp; 

		}


		~ITMTrackingState(void)
		{
			delete pointCloud;
			delete pose_d;
			delete pose_pointCloud;
		}

		void Reset()
		{
			this->age_pointCloud = -1;
			this->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			this->pose_pointCloud->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			this->trackerResult = TRACKING_GOOD;
		}

		// Suppress the default copy constructor and assignment operator
		ITMTrackingState(const ITMTrackingState&);
		ITMTrackingState& operator=(const ITMTrackingState&);
	};
}
