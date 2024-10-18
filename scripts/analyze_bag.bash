#!/bin/bash
source $(ros2 pkg prefix aic_tools)/.venv/analyze_bag/bin/activate
python3 $(ros2 pkg prefix aic_tools)/lib/aic_tools/analyze_bag.py $@
