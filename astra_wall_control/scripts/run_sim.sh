#!/usr/bin/env bash
set -euo pipefail

vehicle="${1:-iris_mid360}"
package_share="${4:-}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${package_share}" ]]; then
  # Installed layout: <prefix>/lib/astra_wall_control/run_px4_gazebo.sh
  if [[ -d "${script_dir}/../../share/astra_wall_control" ]]; then
    package_share="$(cd "${script_dir}/../.." && pwd)/share/astra_wall_control"
  else
    # Source layout: <ws>/src/astra_wall_control/scripts/run_px4_gazebo.sh
    package_share="$(cd "${script_dir}/.." && pwd)"
  fi
fi
# ROS 2 specific copy of the Astra world.  The original file in
# simulation/astra_gazebo_worlds is shared with the ROS 1 workspace, so the
# ROS 2 side keeps its own copy whose <physics> profile is named
# "default_physics"; PX4's Gazebo lockstep plugin selects that profile by name
# and refuses to step the world when it is missing.
world="${2:-${package_share}/worlds/astra_example.world}"
gui="${3:-true}"
lidar_downsample="${5:-1}"

px4_dir="${PX4_DIR:-/home/li/PX4-Autopilot}"
px4_build="${PX4_BUILD_DIR:-${px4_dir}/build/px4_sitl_default}"
astra_root="${ASTRA_ROOT:-/home/li/AstraDroneOpen}"
rootfs="${ASTRA_WALL_ROOTFS:-${astra_root}/AstraDrone_ros2_ws/log/wall_px4_sitl}"

px4_bin="${px4_build}/bin/px4"
model_file="${package_share}/models/${vehicle}/${vehicle}.sdf"
if [[ ! -f "${model_file}" ]]; then
  model_file="${px4_dir}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${vehicle}/${vehicle}.sdf"
fi

if [[ ! -x "${px4_bin}" ]]; then
  echo "PX4 SITL binary not found: ${px4_bin}" >&2
  echo "Build it with: cd ${px4_dir} && make px4_sitl_default" >&2
  exit 1
fi
if [[ ! -f "${world}" ]]; then
  echo "Gazebo world not found: ${world}" >&2
  exit 1
fi
if [[ ! -f "${model_file}" ]]; then
  echo "PX4 model not found: ${model_file}" >&2
  exit 1
fi

# Gazebo's setup script dereferences variables that may be unset, so relax
# `set -u` while sourcing it (and the same holds for any ROS setup file).
set +u
source "${px4_dir}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "${px4_dir}" "${px4_build}"
set -u

# Put the ROS 2 Mid-360 model first. Other Astra and PX4 models keep their
# original locations, so this does not modify or replace the ROS 1 assets.
export GAZEBO_MODEL_PATH="${package_share}/models:${astra_root}/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="${package_share}/../../lib:${GAZEBO_PLUGIN_PATH:-}"
export PX4_SIM_MODEL="gazebo-classic_${vehicle}"
export PX4_SIM_WORLD="${world}"

mkdir -p "${rootfs}"

# Per-run sensor override; keep the shared full-density model unchanged.
/usr/bin/python3 - "${package_share}" "${rootfs}/runtime_models" "${lidar_downsample}" <<'PY_MODEL'
import pathlib, shutil, sys, xml.etree.ElementTree as ET
share, destination = map(pathlib.Path, sys.argv[1:3])
factor = int(sys.argv[3])
if not 1 <= factor <= 8:
    raise SystemExit('lidar_downsample must be an integer from 1 to 8')
model = destination / 'mid360'
model.mkdir(parents=True, exist_ok=True)
for config in (share / 'models' / 'mid360').glob('model.config'):
    shutil.copy2(config, model / config.name)
tree = ET.parse(share / 'models' / 'mid360' / 'mid360.sdf')
tree.find(".//plugin[@name='ros2_livox']/downsample").text = str(factor)
tree.write(model / 'mid360.sdf', encoding='unicode', xml_declaration=True)
PY_MODEL
export GAZEBO_MODEL_PATH="${rootfs}/runtime_models:${GAZEBO_MODEL_PATH}"

# PX4 loads `parameters.bson` from its working directory at boot, i.e. before
# the estimator starts.  Deploying the shipped parameter set here makes EKF2
# boot straight into external-vision mode; setting EKF2_EV_CTRL only after boot
# leaves the EKF2 horizontal state uninitialised (NaN) and arming is refused.
if [[ -f "${package_share}/config/px4_ekf2_ev.bson" ]]; then
  /usr/bin/python3 "${script_dir}/seed_parameters.py" "${package_share}/config/px4_ekf2_ev.bson" "${package_share}/config/flight.json" "${rootfs}/parameters.bson"
fi

gzserver_pid=""
gzclient_pid=""
px4_pid=""

# PX4 keeps an fcntl lock on a per-instance file to reject a second server for
# the same instance.  The lock is inherited by the PX4 task processes (their
# command names start with `px4-`), so a previous SIGKILL of the parent leaves
# the lock held and the next start fails with "PX4 server already running".
# Only processes whose working directory is this workspace's rootfs are killed.
stop_stale_px4() {
  local pid cwd round
  for round in TERM KILL; do
    for pid in $(pgrep '^px4' 2>/dev/null || true); do
      cwd="$(readlink -f "/proc/${pid}/cwd" 2>/dev/null || true)"
      if [[ "${cwd}" == "${rootfs}" ]]; then
        echo "Stopping stale PX4 process ${pid} (${round})"
        kill "-${round}" "${pid}" 2>/dev/null || true
      fi
    done
    sleep 1
  done
}

cleanup() {
  trap - EXIT INT TERM
  for pid in "${px4_pid}" "${gzclient_pid}" "${gzserver_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  stop_stale_px4
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

stop_stale_px4

# Insert the vehicle directly into the world instead of inserting it at runtime
# with `gz model`.  The runtime factory round trip can block forever when the
# Gazebo factory never answers, which leaves PX4 waiting on TCP 4560 with no
# IMU data.  Loading the vehicle with the world is what PX4's own
# `sitl_run.sh` does and it also guarantees the MAVLink plugin is listening on
# TCP 4560 before PX4 starts.
run_world="${rootfs}/astra_run.world"
/usr/bin/python3 - "${world}" "${run_world}" "${vehicle}" <<'PY_WORLD'
import sys, xml.etree.ElementTree as ET
source, destination, vehicle = sys.argv[1:]
tree = ET.parse(source)
include = ET.SubElement(tree.getroot().find('world'), 'include')
ET.SubElement(include, 'uri').text = 'model://' + vehicle
ET.SubElement(include, 'pose').text = '0 0 0.001 0 0 0'
tree.write(destination, encoding='unicode')
PY_WORLD

if ! grep -q "model://${vehicle}" "${run_world}"; then
  echo "Failed to add ${vehicle} to the world ${world}" >&2
  exit 1
fi

gzserver "${run_world}" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so --ros-args --params-file "${package_share}/config/gazebo.yaml" &
gzserver_pid=$!

# Wait until the world has actually loaded before PX4 starts, so the MAVLink
# plugin is already listening when PX4 tries to connect.
gazebo_ready=false
for _ in $(seq 1 120); do
  topic_list="$(gz topic -l 2>/dev/null || true)"
  if [[ "${topic_list}" == *"/model/info"* ]] && \
     [[ "${topic_list}" == *"${vehicle}"* ]]; then
    gazebo_ready=true
    break
  fi
  if ! kill -0 "${gzserver_pid}" 2>/dev/null; then
    echo "gzserver exited before becoming ready" >&2
    exit 1
  fi
  sleep 0.5
done
if [[ "${gazebo_ready}" != true ]]; then
  echo "Timed out while waiting for Gazebo to become ready" >&2
  exit 1
fi
sleep 1

# `stdbuf -oL` keeps PX4's console output line buffered so the startup log
# stays readable while the simulation is running.
(cd "${rootfs}" && stdbuf -oL -eL "${px4_bin}" -d "${px4_build}/etc") &
px4_pid=$!

if [[ "${gui}" == "true" || "${gui}" == "1" ]]; then
  gzclient --verbose --gui-client-plugin libgazebo_user_camera_plugin.so &
  gzclient_pid=$!
fi

wait "${px4_pid}"
