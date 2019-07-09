#!/usr/bin/env bash

python camera.py calibrate
python calibration/calibration.py
rm calibration/calibrate*.jpg
echo please manually copy values into camera.py
