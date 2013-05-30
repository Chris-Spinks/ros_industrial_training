#!/bin/bash

function set_title {
echo -en "\033]2;$1\007"
}

function set_training_unit {

if [ $# -ne 2 ]
then
  echo "Usage: $FUNCNAME UNIT SUBDIR"
  return $E_BADARGS
fi

# get training directory from this file's path
local TRAINING_DIR=$(dirname "$BASH_SOURCE")

# assume function arg is unit ID (e.g. 1.2)
local UNIT=$1    # arg 1 is unitID (e.g. 1.2)
local SUBDIR=$2  # arg 2 is subdir (e.g. work or ans)
local UNIT_DIR=$TRAINING_DIR/$SUBDIR/$UNIT

# check that directory exists
if [ ! -d $UNIT_DIR ]; then
  echo -e "\n\e[00;31m  ERROR: Directory '$UNIT_DIR' does not exist.\e[00m\n"
  return
fi

source ~/.bashrc  # reset ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$UNIT_DIR:$TRAINING_DIR/supplements:$ROS_PACKAGE_PATH
set_title "ROS-I Training Unit $UNIT ($SUBDIR)"
cd $UNIT_DIR
echo -e "\n\e[00;32m  Switching to UNIT $UNIT ($SUBDIR copy)\e[00m\n"

}

function training_unit {
if [ $# -ne 1 ]; then
  echo "Usage: $FUNCNAME UNIT"
else
  set_training_unit $1 work
fi
}

function training_ref {
if [ $# -ne 1 ]; then
  echo "Usage: $FUNCNAME UNIT"
else
  set_training_unit $1 ref
fi
}
