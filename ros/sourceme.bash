#!/usr/bin/env bash

ABSOLUTE_PATH="$(cd "${0%/*}" 2>/dev/null; echo "$PWD"/"${0##*/}")"
SCRIPT_DIR=`dirname "$ABSOLUTE_PATH"`

source devel/setup.bash
export GAZEBO_MODEL_PATH=$SCRIPT_DIR/src/slim_gazebo/models
export GAZEBO_RESOURCE_PATH=$SCRIPT_DIR/src/slim_gazebo/worlds
