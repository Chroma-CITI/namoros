#!/bin/bash

set -eo pipefail

DIR=$(dirname "$0")
cd $DIR/..

export NAMO_DEACTIVATE_TKINTER="TRUE"
export NAMO_DEACTIVATE_RVIZ="TRUE"
python3 -m pytest -s --cov=namoros --cov-fail-under=1 namoros/test