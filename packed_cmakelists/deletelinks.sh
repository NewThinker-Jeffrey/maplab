#! /bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"

## CATKIN_IGNORE 
cmd="rm ${my_loc}/CATKIN_IGNORE"
echo $cmd
$cmd


## maplab_core
module="maplab_core/algorithms"
cmd="rm ${my_loc}/${module}/code"
echo $cmd
$cmd


module="maplab_core/common"
cmd="rm ${my_loc}/${module}/code"
echo $cmd
$cmd

module="maplab_core/aslam_cv2"
cmd="rm ${my_loc}/${module}/code"
echo $cmd
$cmd


module="maplab_core/map-structure"
cmd="rm ${my_loc}/${module}/code"
echo $cmd
$cmd


module="maplab_core/backend"
cmd="rm ${my_loc}/${module}/code"
echo $cmd
$cmd


## console-plugins
module="console-plugins"
cmd="rm ${my_loc}/${module}/code"
echo $cmd
$cmd


