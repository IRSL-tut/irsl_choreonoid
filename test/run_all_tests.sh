#!/bin/bash

set -e

CNOID_VER="$(echo $(find $(dirname $(which choreonoid))/../share -maxdepth 1 -name choreonoid-*) | sed -e 's@.*choreonoid-\(.*\)@\1@g')"

PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-${CNOID_VER}/python

TEST_FILES=$(ls -1 *.py)

for FLS_ in ${TEST_FILES}; do
    echo "######## run ${FLS_} ########"
    python3 ${FLS_}
done
