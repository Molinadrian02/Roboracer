#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /websim_ws/install/setup.bash
exec "$@"
