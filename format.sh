#!/bin/sh

echo --------------------------------------------------
echo "PROJECT: 2022-RAPID-REACT"
echo "MACHINE HOSTNAME: $HOSTNAME"
echo "TASK: FORMAT (SPOTLESS)"
echo "TIME: $(date)"
if ./gradlew spotlessApply > log.txt ; then
    echo "RESULT: PASSED"
    echo --------------------------------------------------
else
    echo "BUILD FAILED. CHECK LOG.TXT FOR INFO."
    echo --------------------------------------------------
fi