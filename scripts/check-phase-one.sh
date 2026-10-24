#!/usr/bin/env bash
# Reproducible simulator-only acceptance. Restarts the demo and moves the rover.
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[[ -n ${TMUX:-} ]] || { echo 'Run inside Thor tmux orbital-work.' >&2; exit 1; }
[[ $ROOT == /home/jensen/.goske/codex/orbital-mechanic ]] || {
  echo 'This acceptance runner is restricted to the Thor simulation workspace.' >&2; exit 1;
}
cd "$ROOT"
ulimit -c 0
source scripts/ros-env.sh
report="$ROOT/runtime/phase-one-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$report"
/usr/bin/python3 - "$report" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path.cwd()
files=sorted([*root.glob('scripts/*'),*root.glob('config/*'),*root.glob('orbital/*.py'),*root.glob('worlds/*.sdf')])
manifest={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
Path(sys.argv[1],'source-sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
fresh_world() {
  if [[ $(systemctl --user show orbital-demo.service --property=LoadState --value) != not-found ]]; then
    bash scripts/stop-thor.sh
  fi
  bash scripts/launch-thor.sh
  sleep 12
  systemctl --user is-active orbital-demo.service
}
# On failure stop the whole demo; a failed test is never silently left driving.
trap 'result=$?; if (( result != 0 )); then bash scripts/stop-thor.sh || true; fi' EXIT
/usr/bin/python3 -m unittest discover -s tests -v > "$report/unit.log" 2>&1
fresh_world
timeout 20 /usr/bin/python3 scripts/check-frames.py > "$report/frames-before.json"
timeout 45 /usr/bin/python3 scripts/check-motion.py > "$report/motion.json"
fresh_world
timeout 160 /usr/bin/python3 scripts/check-operator.py > "$report/operator.json"
fresh_world
timeout 100 /usr/bin/python3 scripts/check-motion-failures.py > "$report/failures.json"
timeout 20 /usr/bin/python3 scripts/check-frames.py > "$report/frames-after.json"
timeout 30 /usr/bin/python3 scripts/probe-sensors.py > "$report/sensors.json"
printf 'Runtime acceptance passed. Evidence: %s\nFoxglove rendered-view acceptance is separate.\n' "$report"
