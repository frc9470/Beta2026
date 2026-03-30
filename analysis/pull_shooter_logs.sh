#!/usr/bin/env bash
set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-lvuser@roborio-9470-frc.local}"
REMOTE_DIR="${REMOTE_DIR:-/home/lvuser/logs/shooter-char}"
DEST_DIR="${1:-analysis/data/raw}"

mkdir -p "${DEST_DIR}"
before_count="$(find "${DEST_DIR}" -maxdepth 1 -name '*.csv' | wc -l | tr -d ' ')"
scp "${ROBOT_HOST}:${REMOTE_DIR}/*.csv" "${DEST_DIR}/"
after_count="$(find "${DEST_DIR}" -maxdepth 1 -name '*.csv' | wc -l | tr -d ' ')"
copied_count="$((after_count - before_count))"

echo "Pulled ${copied_count} new shooter characterization CSV file(s) into ${DEST_DIR}"
