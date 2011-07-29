The tab "'Run' commands" for the debug should have this:

monitor halt
monitor flash write_image erase unlock Blinky.hex
monitor gdb_breakpoint_override hard
monitor reset
continue
