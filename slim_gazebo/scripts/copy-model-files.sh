#!/usr/bin/env sh

ABSOLUTE_PATH="$(cd "${0%/*}" 2>/dev/null; echo "$PWD"/"${0##*/}")"
SCRIPT_DIR=`dirname "$ABSOLUTE_PATH"`

if [ ! -d ~/.gazebo/models/kiva_pod ]; then
    echo -n 'Copying kiva_pod into local gazebo models repository...'
    mkdir -p ~/.gazebo/models
    cp -r $SCRIPT_DIR/../models/kiva_pod ~/.gazebo/models
    echo 'done!'
fi



