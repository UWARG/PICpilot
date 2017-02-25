#!/bin/bash

# Run the unit tests and generate all the required gcov files
ceedling gcov:all

# Make the coverage directory
mkdir -p coverage

# Run lcov to clear the counters
#lcov --directory build/gcov/out --zerocounters

# Capturing the current coverage state to a file
lcov --directory ./build/gcov/out --capture --output-file ./coverage/app.info

# generate the html output
genhtml ./coverage/app.info -o ./coverage


