#!/bin/sh

echo --------------------------------------------------
echo "PROJECT: 2022-RAPID-REACT"
echo "MACHINE HOSTNAME: $HOSTNAME"
echo "TASK: BUILD"
echo "TIME: $(date)"
if ./gradlew build > log.txt ; then
    echo "RESULT: PASSED"
    echo --------------------------------------------------
else
    echo "BUILD FAILED. CHECK LOG.TXT FOR INFO."
    echo --------------------------------------------------
fi