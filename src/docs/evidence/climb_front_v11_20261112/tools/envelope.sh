#!/bin/bash
# usage: envelope.sh cfg tag "dx dy yaw;..."
CFG=$(realpath $1); TAG=$2; OUT=/home/artrc/grasp_sync_backups/claude_front_v10_stagewise_20261110/gpu_$TAG; mkdir -p $OUT
cd /home/artrc/grasp_sync_backups/claude_front_v10_stagewise_20261110
IFS=";" read -ra LIST <<< "$3"
for e in "${LIST[@]}"; do
  set -- $e; t="dx${1}_dy${2}_yaw${3}"
  python3 - "$CFG" "$OUT/cfg_$t.json" $1 $2 <<'PY'
import json,sys
c=json.load(open(sys.argv[1])); c['xiaolan_translation'][0]-=float(sys.argv[3])/1000; c['xiaolan_translation'][1]-=float(sys.argv[4])/1000
json.dump(c,open(sys.argv[2],'w'))
PY
  YAW_DEG=$(python3 -c "print(-float('$3'))") timeout 500 python3 yaw_sim.py --headless --climb-start --climb-config $OUT/cfg_$t.json --climb-speed 1 --climb-joint-speed 1.8 --climb-metrics $OUT/m_$t.json > $OUT/log_$t.txt 2>&1
  echo "$t: $(grep -E 'PREVIEW VERDICT' $OUT/log_$t.txt | cut -c20-45) $(grep -oE 'FAILED [A-Z_0-9]+' $OUT/log_$t.txt | tail -1)"
done
