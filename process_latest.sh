#!/bin/bash

FILE=$(ls ../kamera_rpi/output_* | tail -n 1)

bin/main $FILE
