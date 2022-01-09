#!/bin/sh

echo --------------------------------------------------
echo "PROJECT: 2022-RAPID-REACT"
echo "MACHINE HOSTNAME: $HOSTNAME"
echo "TASK: DEPLOY"
echo "TIME: $(date)"
if ./gradlew deploy > log.txt ; then
    echo "RESULT: PASSED"
    echo --------------------------------------------------
else
    echo "BUILD FAILED. CHECK LOG.TXT FOR INFO."
    echo --------------------------------------------------
fi