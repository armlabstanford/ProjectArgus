#!/bin/bash
# Environment for Project Argus realtime IK (ROS 2 Humble + OpenSim 4.6).
# Source from any directory:  source /path/to/ProjectArgus/source_env.sh

_ARGUS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

# OpenSim 4.6 was built from source into ~/opensim-core (non-FHS layout).
export OpenSim_DIR="${OpenSim_DIR:-$HOME/opensim-core/cmake}"
export CMAKE_PREFIX_PATH="$HOME/opensim-core:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/opensim-core/sdk/lib:$HOME/opensim-core/sdk/Simbody/lib:${LD_LIBRARY_PATH:-}"
export PATH="$HOME/opensim-core/bin:$PATH"

if [ -f "${_ARGUS_ROOT}/install/setup.bash" ]; then
  source "${_ARGUS_ROOT}/install/setup.bash"
fi

unset _ARGUS_ROOT
