The tab "'Run' commands" for the debug should have this:

gdb_breakpoint_override
monitor reset
monitor halt
monitor flash write_image erase unlock FLASH_RUN/project.elf
monitor reset
monitor halt
