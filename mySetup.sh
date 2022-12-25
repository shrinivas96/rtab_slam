#!/bin/bash

source ./devel/setup.bash
export TURTLEBOT3_MODEL=${1-waffle}
echo "$TURTLEBOT3_MODEL model activated"
