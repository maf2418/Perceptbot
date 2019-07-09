#!/usr/bin/env bash 

python -m coverage erase 
python -m coverage run -a --source=./ --branch test/test_DriveChain.py 
python -m coverage run -a --source=./ --branch test/test_MotionController.py 
python -m coverage run -a --source=./ --branch test/test_Odometry.py 
python -m coverage run -a --source=./ --branch test/test_PIDController.py 
python -m coverage run -a --source=./ --branch test/test_Utilities.py 
python -m coverage run -a --source=./ --branch test/test_WheelEncoder.py 
python -m coverage report -m
