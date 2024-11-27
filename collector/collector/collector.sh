#!/bin/bash

source ~/.bashrc
conda activate airexo
export DISPLAY=:1
cd /home/grange/Program/act_arm/collector/
cfg=$1
tid=$2
sid=$3
uid=$4
python collector/collector.py --cfg $cfg --task $tid --scene $sid --user $uid
