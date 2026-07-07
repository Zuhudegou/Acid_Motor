#!/usr/bin/env bash
set -u

elf="${1:-}"

if [ -z "${elf}" ]; then
    echo "usage: $0 <firmware.elf>"
    exit 2
fi

if [ ! -f "${elf}" ]; then
    echo "firmware not found: ${elf}"
    exit 2
fi

echo "CMSIS-DAP power-cycle recovery"
echo "1) Disconnect motor/main power if possible; keep only MCU power path."
echo "2) Power the target OFF."
echo "3) Leave this command running, then power the target ON."
echo "4) If BOOT0 is available, hold BOOT0 high while powering ON."
echo

attempt=1
while true; do
    echo "Attempt ${attempt}: waiting for a connectable target..."
    openocd \
        -f interface/cmsis-dap.cfg \
        -f target/stm32g4x.cfg \
        -c "gdb_port disabled" \
        -c "tcl_port disabled" \
        -c "telnet_port disabled" \
        -c "adapter speed 50" \
        -c "reset_config none" \
        -c "cortex_m reset_config sysresetreq" \
        -c "init" \
        -c "halt" \
        -c "program ${elf} verify" \
        -c "reset run" \
        -c "shutdown"

    status=$?
    if [ "${status}" -eq 0 ]; then
        echo "Flash completed."
        exit 0
    fi

    attempt=$((attempt + 1))
    sleep 0.25
done
