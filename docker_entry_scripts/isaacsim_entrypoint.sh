#!/bin/bash

source /opt/ros/humble/setup.bash

if [ "$USE_SIM" == "True" ]; then
    ./runheadless.native.sh --reset-data
fi

exec bash
