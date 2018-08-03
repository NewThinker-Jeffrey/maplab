#!/usr/bin/env bash

batch_control_file_and_other_arguments=$@
echo "batch_control_file_and_other_arguments: ${batch_control_file_and_other_arguments}"
cur_dir="$(pwd)"
echo "cur_dir:" ${cur_dir}
my_loc="$(cd "$(dirname $0)" && pwd)"
cd ${my_loc}/../../../../../devel
echo devel dir: `pwd`
source setup.bash
cd ${cur_dir}
cmd="rosrun maplab_console batch_runner --batch_control_file ${batch_control_file_and_other_arguments}"
echo $cmd
$cmd
