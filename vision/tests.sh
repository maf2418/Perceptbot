#!/usr/bin/env bash

python -m coverage erase
python -m coverage run -a --source=src/ --branch tests/tests_ssdlocalization.py
python3 -m coverage run -a --source=src/ --branch tests/tests_obj_detection.py
python -m coverage report -m
