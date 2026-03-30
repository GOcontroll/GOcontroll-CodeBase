#!/bin/bash
# =============================================================================
# update_xcp_a2l.sh — patch XCP a2l addresses from ELF symbol table
#
# Usage:
#   ./update_xcp_a2l.sh <elf> <a2l_template> <a2l_output>
#
# Example:
#   ./GOcontroll-CodeBase/examples/update_xcp_a2l.sh \
#       build/app.elf \
#       GOcontroll-CodeBase/examples/xcp_basic.a2l \
#       xcp_basic_connected.a2l
#
# The script reads the ELF symbol table with 'nm', looks up every variable
# whose name appears as a __ADDR_<name>__ placeholder in the a2l template,
# and writes the resolved address into the output a2l.
#
# All XCP variables (M_ and C_ prefixed) are detected automatically — no
# manual editing of this script is needed when variables are added or removed.
# =============================================================================

set -euo pipefail

ELF="${1:-}"
TEMPLATE="${2:-}"
OUTPUT="${3:-}"

if [[ -z "$ELF" || -z "$TEMPLATE" || -z "$OUTPUT" ]]; then
    echo "Usage: $0 <elf> <a2l_template> <a2l_output>" >&2
    exit 1
fi

if [[ ! -f "$ELF" ]]; then
    echo "ERROR: ELF file not found: $ELF" >&2
    exit 1
fi

if [[ ! -f "$TEMPLATE" ]]; then
    echo "ERROR: a2l template not found: $TEMPLATE" >&2
    exit 1
fi

# Start with a copy of the template
cp "$TEMPLATE" "$OUTPUT"

# Find all placeholders in the template: __ADDR_<varname>__
PLACEHOLDERS=$(grep -oP '__ADDR_\K[A-Za-z0-9_]+(?=__)' "$TEMPLATE" || true)

if [[ -z "$PLACEHOLDERS" ]]; then
    echo "WARNING: no __ADDR_*__ placeholders found in template." >&2
    exit 0
fi

UPDATED=0
MISSING=0

while IFS= read -r varname; do
    # Look up address in ELF symbol table (nm posix format: name type addr size)
    ADDR=$(nm --format=posix "$ELF" 2>/dev/null \
           | awk -v v="$varname" '$1 == v { print $3; exit }')

    if [[ -z "$ADDR" ]]; then
        echo "  WARNING: symbol '$varname' not found in ELF — placeholder left unchanged" >&2
        MISSING=$((MISSING + 1))
        continue
    fi

    # Convert to 32-bit hex address (a2l uses 32-bit even on 64-bit systems)
    ADDR32=$(printf "0x%08X" "0x${ADDR}")

    # Replace placeholder in output file
    sed -i "s/__ADDR_${varname}__/${ADDR32}/g" "$OUTPUT"

    echo "  ${varname} = ${ADDR32}"
    UPDATED=$((UPDATED + 1))

done <<< "$PLACEHOLDERS"

echo ""
echo "a2l updated: $OUTPUT"
echo "  $UPDATED variable(s) resolved, $MISSING missing"

if [[ $MISSING -gt 0 ]]; then
    exit 1
fi
