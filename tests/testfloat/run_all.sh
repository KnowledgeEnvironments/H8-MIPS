#!/bin/bash
# run_all.sh — Run TestFloat vectors through the FPU Verilator model
# Usage: run_all.sh <testfloat_gen> <fpu_exe> [--level N] [--max-fails N]

set -e

TESTFLOAT_GEN="${1:?Usage: run_all.sh <testfloat_gen> <fpu_exe>}"
FPU_EXE="${2:?Usage: run_all.sh <testfloat_gen> <fpu_exe>}"
LEVEL="${LEVEL:-1}"
MAX_FAILS="${MAX_FAILS:-10}"

PASS=0
FAIL=0
SKIP=0
TOTAL=0
FAILED_OPS=""

# Rounding mode names for testfloat_gen
# MIPS: RM_RN=0, RM_RZ=1, RM_RP=2(+inf), RM_RM=3(-inf)
# TestFloat: rmin=toward -inf, rmax=toward +inf
RMODES=("rnear_even" "rminMag" "rmin" "rmax")
RMODE_IDS=(0 1 3 2)
RMODE_NAMES=("RNE" "RTZ" "RDN" "RUP")

# Operations to test
ARITH_OPS="f32_add f32_sub f32_mul f32_div f32_sqrt f64_add f64_sub f64_mul f64_div f64_sqrt"
CVT_FLOAT_OPS="f32_to_f64 f64_to_f32"
CVT_INT_OPS="f32_to_i32 f32_to_i64 f64_to_i32 f64_to_i64 i32_to_f32 i32_to_f64 i64_to_f32 i64_to_f64"
CMP_OPS="f32_eq f32_le f32_lt f64_eq f64_le f64_lt f32_eq_signaling f32_le_quiet f32_lt_quiet f64_eq_signaling f64_le_quiet f64_lt_quiet"

run_one() {
    local op="$1"
    local rmode_id="$2"
    local rmode_flag="$3"
    local rmode_name="$4"

    TOTAL=$((TOTAL + 1))

    # Comparisons are rounding-mode independent — only run once (rmode 0)
    case "$op" in
        *_eq|*_le|*_lt|*_eq_signaling|*_le_quiet|*_lt_quiet)
            if [ "$rmode_id" != "0" ]; then
                SKIP=$((SKIP + 1))
                return
            fi
            ;;
    esac

    local log="/tmp/fpu_tf_${op}_${rmode_name}.log"

    if "$TESTFLOAT_GEN" -level "$LEVEL" -"$rmode_flag" "$op" 2>/dev/null | \
       "$FPU_EXE" --op="$op" --rmode="$rmode_id" --max-fails="$MAX_FAILS" > "$log" 2>&1; then
        PASS=$((PASS + 1))
        result=$(tail -1 "$log")
        printf "  PASS  %-25s %s  %s\n" "$op" "$rmode_name" "$result"
    else
        FAIL=$((FAIL + 1))
        FAILED_OPS="$FAILED_OPS  $op/$rmode_name"
        result=$(tail -1 "$log")
        printf "  FAIL  %-25s %s  %s\n" "$op" "$rmode_name" "$result"
        # Print first few failures
        grep "^FAIL" "$log" | head -3 >&2
    fi
}

echo "=== FPU TestFloat Verification (level $LEVEL) ==="
echo ""

echo "--- Arithmetic ---"
for op in $ARITH_OPS; do
    for i in 0 1 2 3; do
        run_one "$op" "${RMODE_IDS[$i]}" "${RMODES[$i]}" "${RMODE_NAMES[$i]}"
    done
done

echo ""
echo "--- Conversions (float<->float) ---"
for op in $CVT_FLOAT_OPS; do
    for i in 0 1 2 3; do
        run_one "$op" "${RMODE_IDS[$i]}" "${RMODES[$i]}" "${RMODE_NAMES[$i]}"
    done
done

echo ""
echo "--- Conversions (float<->int) ---"
for op in $CVT_INT_OPS; do
    for i in 0 1 2 3; do
        run_one "$op" "${RMODE_IDS[$i]}" "${RMODES[$i]}" "${RMODE_NAMES[$i]}"
    done
done

echo ""
echo "--- Comparisons ---"
for op in $CMP_OPS; do
    for i in 0 1 2 3; do
        run_one "$op" "${RMODE_IDS[$i]}" "${RMODES[$i]}" "${RMODE_NAMES[$i]}"
    done
done

echo ""
echo "=== Summary ==="
echo "Total: $TOTAL  Pass: $PASS  Fail: $FAIL  Skip: $SKIP"
if [ $FAIL -gt 0 ]; then
    echo "Failed operations:$FAILED_OPS"
    exit 1
fi
echo "All tests passed."
exit 0
