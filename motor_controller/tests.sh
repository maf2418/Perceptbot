#!/usr/bin/env bash

python -m coverage erase
python -m coverage run -a --source=src/ --branch test/gpio_test.py
python -m coverage run -a --source=src/ --branch test/mover_test.py
python -m coverage report -m
