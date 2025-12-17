#!/usr/bin/env bash
set -euo pipefail

# Publishes two /pwm_test messages in sequence.
# 1) motor_index: [] , pwms: [0.2]
# 2) motor_index: [] , pwms: []
#
# Usage:
#   ./pwm_test_sequence.sh            # default delay
#   DELAY_SEC=0.0 ./pwm_test_sequence.sh

DELAY_SEC="${DELAY_SEC:-0.5}"

rostopic pub -1 /pwm_test spinal/PwmTest "{motor_index: [], pwms: [0.2]}"

# Optional delay between messages
if [[ "${DELAY_SEC}" != "0" && "${DELAY_SEC}" != "0.0" ]]; then
  sleep "${DELAY_SEC}"
fi

rostopic pub -1 /pwm_test spinal/PwmTest "{motor_index: [], pwms: []}"
