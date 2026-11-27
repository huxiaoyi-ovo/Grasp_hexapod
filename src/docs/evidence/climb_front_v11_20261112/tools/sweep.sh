#!/bin/bash
# robot start error e=(dx,dy) mm relative to Xiaolan  ->  xiaolan_translation = nominal - e
cd /home/artrc/Grasp_hexapod
CFG=src/grasp_hexapod_control/config/climb_front_optimized.json
OUT=/home/artrc/grasp_sync_backups/claude_front_v10_stagewise_20261110/sweep
mkdir -p $OUT
for e in "-10 0" "10 0" "-20 0" "20 0" "0 15" "0 -15"; do
  set -- $e; tag="dx${1}_dy${2}"
  python3 - "$CFG" "$OUT/cfg_$tag.json" $1 $2 <<'PY'
import json,sys
c=json.load(open(sys.argv[1])); dx,dy=float(sys.argv[3])/1000,float(sys.argv[4])/1000
c['xiaolan_translation'][0]-=dx; c['xiaolan_translation'][1]-=dy
json.dump(c,open(sys.argv[2],'w'))
PY
  timeout 400 python3 src/grasp_hexapod_control/scripts/run_sim.py --headless --climb-start --climb-config $OUT/cfg_$tag.json --climb-speed 1 --climb-joint-speed 1.8 --climb-metrics $OUT/m_$tag.json > $OUT/log_$tag.txt 2>&1
  echo "$tag exit $? $(grep -E 'PREVIEW VERDICT|DONE at|FAILED|state=FAILED' $OUT/log_$tag.txt | tail -1 | cut -c1-120)"
done
