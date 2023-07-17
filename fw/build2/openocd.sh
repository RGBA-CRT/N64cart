#!/bin/bash
/mingw64/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c 'bindto 0.0.0.0'
