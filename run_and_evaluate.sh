#!/bin/bash
#Shell script to run InfiniTAM on a bunch of datasets 

#Specify path to the InfiniTAM_cli executable
InfiniTAM_executable_path="/Users/prakrutigogia/Documents/CMU/Academics/Fall_17/Geometry/Project/InfiniTAM/InfiniTAM/build/Apps/InfiniTAM_cli"
executable_name='./InfiniTAM_cli'


#Note run 
#mogrify -format ppm frame-000*.color.png
#mogrify -format pgm frame-000*.depth.png
#modify the calibration file - TODO write a python script to automatically generate a calib.txt in the right format

#

#change to folder containing executable
cd $InfiniTAM_executable_path

calib_filename='calib.txt'
rgb_filename='frame-%06i.color.ppm'
depth_filename='frame-%06i.depth.pgm'

#modify this to add whichever datasets you're using for evaluation
declare -a dataset_name_array=("minion_data/" "umbrella_data/")

#Specify path to the folder containing the datasets (absolute path) the mesh will be stored here
path_to_datasets='/Users/prakrutigogia/Documents/CMU/Academics/Fall_17/Geometry/Project/'
for i in "${dataset_name_array[@]}"
do
	dataset_name="$i"
	dataset_path=$path_to_datasets$dataset_name

	calibfile=$dataset_path$calib_filename

	rgbfile=$dataset_path$rgb_filename

	depthfile=$dataset_path$depth_filename
	
	echo "*******Started running ..**********" 
	echo $dataset_name
	$executable_name $calibfile $rgbfile $depthfile $dataset_path
	echo "*******Finished running ..**********" 

	#TODO : convert all .stl files to .ply
	

done

