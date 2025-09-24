#!/bin/bash

set -eo pipefail

DIR=$(dirname "$0")
cd $DIR/..

cd docs
sphinx-apidoc -o source/ ../namoros/namoros
make clean
make html
