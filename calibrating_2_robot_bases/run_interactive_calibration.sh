#!/usr/bin/env bash
set -euo pipefail

calibration_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="${calibration_dir}"
while [ ! -f "${workspace_dir}/pixi.toml" ]; do
  parent_dir="$(dirname -- "${workspace_dir}")"
  if [ "${parent_dir}" = "${workspace_dir}" ]; then
    echo "Could not find the enclosing ur_motion_stack/pixi.toml." >&2
    exit 1
  fi
  workspace_dir="${parent_dir}"
done

cd "${workspace_dir}"
export PYTHONPATH="${calibration_dir}${PYTHONPATH:+:${PYTHONPATH}}"

exec pixi run /usr/bin/python3 \
  -m robot_base_calibration.interactive_calibration "$@"
