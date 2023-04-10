#!/bin/bash

set -e

PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-1.8/python

TEST_FILES=$(ls -1 *.py)

for FLS_ in ${TEST_FILES}; do
    python3 ${FLS_}
done
