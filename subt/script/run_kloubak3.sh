#!/bin/bash

python -m subt run config/kloubak3-subt-estop-lora.json --side left --speed 0.7 --timeout 300
python -m osgar.record config/test-lora.json
