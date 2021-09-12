#!/bin/bash
make flash
gdb-multiarch -ex "target extended-remote | \
       openocd -c \"gdb_port pipe; log_output util/openocd.log\" -f util/stm32h7_stlink.cfg" -ex "tui enable" build/base.elf
