#!/usr/bin/env bash
# Sourced by pixi on environment activation.
#
# RoboStack's turtlebot3_gazebo conda package does not register its models with
# Gazebo (unlike the apt package), so worlds referencing model://turtlebot3_*
# would load empty. Point GAZEBO_MODEL_PATH at the package's models directory.
export GAZEBO_MODEL_PATH="${CONDA_PREFIX}/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"

# Gazebo Classic 11 defaults this to http://models.gazebosim.org, a server
# retired in the Fuel migration. gzserver makes a blocking call to it on startup
# that never returns, so Gazebo hangs on the splash screen. Disable the online
# database; every model our worlds reference is already installed locally and
# resolved via GAZEBO_MODEL_PATH above.
export GAZEBO_MODEL_DATABASE_URI=""
