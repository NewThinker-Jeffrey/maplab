#! /bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"


## maplab_core
maplab_relative_dir="../../.."

src_module=algorithms
dst_module="maplab_core/${src_module}"
cmd="ln -s ${maplab_relative_dir}/${src_module} ${my_loc}/${dst_module}/code"
echo $cmd
$cmd

src_module="common"
dst_module="maplab_core/${src_module}"
cmd="ln -s ${maplab_relative_dir}/${src_module} ${my_loc}/${dst_module}/code"
echo $cmd
$cmd


src_module="aslam_cv2"
dst_module="maplab_core/${src_module}"
cmd="ln -s ${maplab_relative_dir}/${src_module} ${my_loc}/${dst_module}/code"
echo $cmd
$cmd

src_module="map-structure"
dst_module="maplab_core/${src_module}"
cmd="ln -s ${maplab_relative_dir}/${src_module} ${my_loc}/${dst_module}/code"
echo $cmd
$cmd

src_module="backend"
dst_module="maplab_core/${src_module}"
cmd="ln -s ${maplab_relative_dir}/${src_module} ${my_loc}/${dst_module}/code"
echo $cmd
$cmd


## console-plugins
maplab_relative_dir="../.."

src_module="console-plugins"
dst_module="${src_module}"
cmd="ln -s ${maplab_relative_dir}/${src_module} ${my_loc}/${dst_module}/code"
echo $cmd
$cmd

