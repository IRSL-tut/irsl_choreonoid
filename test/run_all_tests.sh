#!/bin/bash

set -e

PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-2.0/python

TEST_FILES=$(ls -1 *.py)

for FLS_ in ${TEST_FILES}; do
    echo "######## run ${FLS_} ########"
    python3 ${FLS_}
done
