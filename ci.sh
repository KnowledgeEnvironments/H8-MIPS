#!/bin/sh
set -eu

cd "$(dirname "$0")"
make test

if [ -n "${CHERITEST_ROOT:-}" ] && [ -n "${R4000_SIM_BIN:-}" ]; then
    make cheritest
fi

if [ -n "${SOFTFLOAT_ROOT:-}" ] && [ -n "${TESTFLOAT_ROOT:-}" ]; then
    make testfloat
fi
