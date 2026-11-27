#!/bin/bash
# usage: gpu_check.sh cfg.json tag   -> visible nominal run, then headless start-offset sweep
CFG=$(realpath $1); TAG=$2; OUT=/home/artrc/grasp_sync_backups/claude_front_v10_stagewise_20261110/gpu_$TAG; mkdir -p $OUT
cd /home/artrc/Grasp_hexapod
timeout 500 python3 src/grasp_hexapod_control/scripts/run_sim.py --climb-start --climb-config $CFG --climb-speed 1 --climb-joint-speed 1.8 --climb-metrics $OUT/m_nominal.json > $OUT/log_nominal.txt 2>&1
echo "nominal: $(grep -E 'PREVIEW VERDICT' $OUT/log_nominal.txt | cut -c1-60) $(grep -E 'DONE at|FAILED ' $OUT/log_nominal.txt | tail -1 | cut -c1-90)"
for e in ${SWEEP:-"-10 0" "-20 0" "20 0"}; do :; done
IFS=";" read -ra LIST <<< "${OFFSETS:--10 0;-20 0;20 0}"
for e in "${LIST[@]}"; do
  set -- $e; t="dx${1}_dy${2}"
  python3 - "$CFG" "$OUT/cfg_$t.json" $1 $2 <<'PY'
import json,sys
c=json.load(open(sys.argv[1])); c['xiaolan_translation'][0]-=float(sys.argv[3])/1000; c['xiaolan_translation'][1]-=float(sys.argv[4])/1000
json.dump(c,open(sys.argv[2],'w'))
PY
  timeout 500 python3 src/grasp_hexapod_control/scripts/run_sim.py --headless --climb-start --climb-config $OUT/cfg_$t.json --climb-speed 1 --climb-joint-speed 1.8 --climb-metrics $OUT/m_$t.json > $OUT/log_$t.txt 2>&1
  echo "$t: $(grep -E 'PREVIEW VERDICT' $OUT/log_$t.txt | cut -c1-60) $(grep -E 'FAILED ' $OUT/log_$t.txt | tail -1 | cut -c1-70)"
done
