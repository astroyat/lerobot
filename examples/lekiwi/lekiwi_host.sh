#!/bin/bash

# Source conda environment if available
if [ -f "/home/target/miniforge3/etc/profile.d/conda.sh" ]; then
    source "/home/target/miniforge3/etc/profile.d/conda.sh"
    conda activate lerobot
fi

cd ~/lerobot
python -m lerobot.robots.lekiwi.lekiwi_host "$@"
