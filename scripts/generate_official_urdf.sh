#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
description_root=${1:-/home/xense/fastiter/franka_description}
output_path=${2:-"${project_root}/models/fr3_franka_hand.urdf"}
xacro_bin=${XACRO_BIN:-/opt/ros/humble/bin/xacro}

source_xacro="${description_root}/robots/fr3/fr3.urdf.xacro"
if [[ ! -f "${source_xacro}" ]]; then
  echo "Official FR3 xacro was not found: ${source_xacro}" >&2
  exit 1
fi
if [[ ! -x "${xacro_bin}" ]]; then
  echo "xacro executable was not found: ${xacro_bin}" >&2
  exit 1
fi

stage_dir=$(mktemp -d /tmp/fr3_description_ament.XXXXXX)
cleanup() {
  rm -rf -- "${stage_dir}"
}
trap cleanup EXIT

mkdir -p "${stage_dir}/share/ament_index/resource_index/packages"
touch "${stage_dir}/share/ament_index/resource_index/packages/franka_description"
ln -s "${description_root}" "${stage_dir}/share/franka_description"
mkdir -p "$(dirname "${output_path}")"

AMENT_PREFIX_PATH="${stage_dir}:${AMENT_PREFIX_PATH:-/opt/ros/humble}" \
  "${xacro_bin}" -o "${output_path}" "${source_xacro}" \
    hand:=true ee_id:=franka_hand with_sc:=false

if command -v check_urdf >/dev/null 2>&1; then
  check_urdf "${output_path}" >/dev/null
fi

echo "Generated official FR3 + Franka Hand URDF: ${output_path}"

