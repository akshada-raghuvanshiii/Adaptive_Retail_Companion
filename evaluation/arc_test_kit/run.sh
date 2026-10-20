#!/usr/bin/env bash
# One-word commands for the ARC simulation tests (plan groups A-C). See README.md.
# Every test first records a snapshot of the system (configs, code hashes, map,
# controller, odometry, BT mode, Gazebo speed) and writes start/finish entries
# to results/LAB_NOTEBOOK.md. Extra arguments are passed to the runner and
# override the preset, e.g.  ./run.sh a2 --lists 10   or   ./run.sh a2 --start-at 12
cd "$(dirname "$0")"
source ./env.sh 2>/dev/null || true   # ROS + Gazebo kept on localhost (see env.sh)
OUT=${OUT:-results}
mkdir -p "$OUT"
R="python3 arc_tests/mission_runner.py --out $OUT"
TEB_YAML=${TEB_YAML:-$HOME/slam_ws/src/arc_planner/config/teb_controller.yaml}

# Presets: test name -> "label | runner arguments"
preset() {
  case "$1" in
    smoke)  echo "smoke | --pairs 1 --seed 5" ;;
    a2)     echo "teb_icp | --lists 30 --size-range 3-12 --strategies as_entered --seed 7" ;;
    b1)     echo "teb_wheel | --lists 10 --size-range 3-12 --strategies as_entered --seed 7" ;;
    a3-teb) echo "teb_icp_pairs | --pairs 20 --seed 5" ;;
    a3-rpp) echo "rpp_icp_pairs | --pairs 20 --seed 5" ;;
    a4)     echo "teb_icp_boxes | --pairs 20 --seed 9 --pair-min-dist 2.5 --obstacles 1-3" ;;
    c2)     echo "teb_icp_c2 | --sizes 5,8,12 --per-size 10 --strategies as_entered,nn_dijkstra,exact --seed 21" ;;
    # phase 2 - same 30 lists and seed as a2; start B1 with the progress variant first (./run.sh make-progress)
    a2-pc)    echo "teb_icp_pc | --lists 30 --size-range 3-12 --strategies as_entered --seed 7" ;;
    a2-final) echo "teb_icp_pc_final | --lists 30 --size-range 3-12 --strategies exact --seed 7" ;;
    *)      return 1 ;;
  esac
}

# run_test NAME extra-args... : snapshot, run, summarise
run_test() {
  local name=$1; shift
  local p; p=$(preset "$name") || { echo "unknown test $name"; exit 1; }
  local label=${p%% |*} args=${p#*| }
  local cmd="$R --label $label $args $*"
  bash arc_tests/snapshot.sh "$OUT" "$name" "$cmd" >/dev/null
  echo "== $name (label $label). Snapshot saved; notebook: $OUT/LAB_NOTEBOOK.md"
  echo "   $cmd"
  $cmd
  local rc=$?
  python3 arc_tests/summarize_label.py "$OUT" "$label" "$name"
  return $rc
}

case "$1" in
  probe)    python3 arc_tests/probe.py --out "$OUT" "${@:2}"; rc=$?
            { echo -e "\n## $(date '+%Y-%m-%d %H:%M') - probe\n\n\`\`\`"
              python3 -c "import json;[print(f'[{s}] {w} {d}') for s,w,d in json.load(open('$OUT/probe.json'))['report']]" 2>/dev/null
              echo '```'; } >> "$OUT/LAB_NOTEBOOK.md"
            exit $rc ;;
  smoke|a2|b1|a3-teb|a3-rpp|a4|c2|a2-pc|a2-final)
            run_test "$1" "${@:2}" ;;
  plan)     # ./run.sh plan a2  : print the missions a preset would run (needs the map; drives nothing)
            p=$(preset "${2:-a2}") || { echo "unknown test $2"; exit 1; }
            $R --label "${p%% |*}" ${p#*| } --dry-run "${@:3}" ;;
  c1)       [ -f "$OUT/map.npz" ] || python3 arc_tests/save_map.py "$OUT"
            python3 arc_tests/ordering_benchmark.py --map "$OUT/map.npz" --out "$OUT" "${@:2}" && \
            echo -e "\n## $(date '+%Y-%m-%d %H:%M') - c1 ordering benchmark (offline)\n\n- Command: \`ordering_benchmark.py ${*:2}\`\n- Map: $OUT/map.npz (md5 $(md5sum "$OUT/map.npz" | cut -c1-12))\n- Output: ordering_benchmark.csv" >> "$OUT/LAB_NOTEBOOK.md" ;;
  footprint) # would a 1000 x 610 mm supermarket cart fit this store? offline, safe during runs
            [ -f "$OUT/map.npz" ] || python3 arc_tests/save_map.py "$OUT"
            python3 arc_tests/footprint_check.py --map "$OUT/map.npz" --out "$OUT" "${@:2}" | tee "$OUT/footprint_check.txt" && \
            echo -e "\n## $(date '+%Y-%m-%d %H:%M') - footprint check (offline)\n\n\`\`\`\n$(cat "$OUT/footprint_check.txt")\n\`\`\`" >> "$OUT/LAB_NOTEBOOK.md" ;;
  c1-checkout)  # ./run.sh c1-checkout X,Y : C1 with every route ending at the checkout (offline, safe during runs)
            [ -n "$2" ] || { echo "usage: ./run.sh c1-checkout X,Y   (checkout standing point in map coordinates)"; exit 1; }
            [ -f "$OUT/map.npz" ] || python3 arc_tests/save_map.py "$OUT"
            python3 arc_tests/ordering_benchmark.py --map "$OUT/map.npz" --out "$OUT" --end "$2" "${@:3}" | tee "$OUT/ordering_benchmark_checkout.txt" && \
            echo -e "\n## $(date '+%Y-%m-%d %H:%M') - c1-checkout (offline, routes end at $2)\n\n\`\`\`\n$(cat "$OUT/ordering_benchmark_checkout.txt")\n\`\`\`" >> "$OUT/LAB_NOTEBOOK.md" ;;
  make-rpp) python3 arc_tests/make_controller_variant.py --in "$TEB_YAML" --out configs/rpp_controller.yaml --type rpp ;;
  make-progress) python3 arc_tests/make_controller_variant.py --in "$TEB_YAML" --out configs/teb_progress.yaml --type progress "${@:2}" ;;
  status)   python3 - "$OUT" <<'EOF'
import csv, os, sys
out = sys.argv[1]
p = os.path.join(out, "missions.csv")
if not os.path.exists(p):
    sys.exit("no missions yet")
rows = list(csv.DictReader(open(p)))
labels = {}
for r in rows:
    labels.setdefault(r["label"], []).append(r)
for lab, ms in labels.items():
    done = sum(int(m["completed"]) for m in ms)
    print(f"{lab:16s} missions logged {len(ms):3d}, completed {done:3d}   (resume with --start-at {len(ms)})")
EOF
            ;;
  note)     shift; echo "- $(date '+%H:%M') NOTE: $*" >> "$OUT/LAB_NOTEBOOK.md"; echo "noted" ;;
  shot)     f="$OUT/run_logs/screenshot_$(date +%Y%m%d_%H%M%S).png"; mkdir -p "$OUT/run_logs"
            gnome-screenshot -f "$f" 2>/dev/null && echo "- $(date '+%H:%M') SCREENSHOT: $f ${*:2}" >> "$OUT/LAB_NOTEBOOK.md" && echo "saved $f" \
              || echo "gnome-screenshot not available; press PrtSc and save to $OUT/run_logs/" ;;
  report)   python3 arc_tests/analyze.py "$OUT" --compare teb_icp_pairs rpp_icp_pairs --compare teb_icp teb_wheel \
              --compare teb_icp teb_icp_pc --compare teb_icp teb_icp_pc_final --compare teb_icp_pc teb_icp_pc_final
            echo "Report: $OUT/report/results.md" ;;
  void)     # ./run.sh void LABEL "reason" : set a bad run's rows aside (kept in results/void/)
            python3 arc_tests/void_label.py "$OUT" "$2" "${@:3}" ;;
  clear-boxes) python3 -c "
import sys; sys.path.insert(0, 'arc_tests')
from rosio import Gz
g = Gz('${WORLD:-arc_store}'); left = g.test_boxes() or []
stuck = g.sweep_test_boxes() if left else []
print(f'test boxes found {len(left)}, removed {len(left) - len(stuck)}' + (f', STILL THERE: {stuck}' if stuck else ''))
sys.exit(1 if stuck else 0)" ;;
  pack)     z="arc_results_$(date +%Y%m%d_%H%M).zip"; zip -qr "$z" "$OUT" && echo "made $z - send this to Claude" ;;
  *) echo "usage: ./run.sh probe|smoke|a2|b1|a3-teb|make-rpp|a3-rpp|a4|c1|c1-checkout X,Y|footprint|c2|make-progress|a2-pc|a2-final|plan TEST|status|note \"text\"|void LABEL \"why\"|clear-boxes|shot|report|pack"; exit 1 ;;
esac
