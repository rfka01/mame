# Local test tools (never for a pull request)

## ljhost (src/mame/hp/ljhost.cpp)

Test host for printers: sends a file from the bitbanger to a printer on the
Centronics port (compatibility mode, then IEEE 1284 nibble mode reverse reads)
or on the RS-232 port (9600 8N1, starts sending after 35 s).

Build together with the printer sources, e.g.

    make SUBTARGET=ljhost SOURCES=src/mame/hp/ljhost.cpp,src/devices/bus/centronics/hp_lj3p.cpp \
         USE_QTDEBUG=0 NO_USE_PIPEWIRE=1 -j2

(luaengine.cpp needs -j1 on machines with little memory.)

Run:

    ./ljhost ljhost -rompath ROMS -centronics fx80 -bitb local-tools/testfiles/u7.prn \
        -video none -sound none -nothrottle -seconds_to_run 60 -snapshot_directory OUT

Environment variables:

- LJDELAY=s   wait s seconds of machine time before sending (printers that drop
              bytes during their power-on initialisation, e.g. FX-880)
- LJSERIAL=1  use the RS-232 port instead of Centronics (e.g. -serial iwriter2)
- LJEXT=hh    after 200 ms idle negotiate IEEE 1284 with extensibility byte hh
- LJDBG=1     print BUSY, PERROR, SELECT and FAULT changes
- LJBAUD=n    serial baud rate of the test host (default 9600)
- LJSTART=s   start sending on the serial port at s seconds (default 35)
- LJBREAKSTART=a, LJBREAK=b  hold the serial line low (break) from a to b seconds

Printed pages land as PNG files in the snapshot directory.

## selftest.lua

Autoboot script: sets the "... held at power on" configuration to KEYVAL and
presses the printer's power switch at 2 s.

    KEYVAL=2 ./ljhost ljhost -centronics fx80 ... -autoboot_script local-tools/selftest.lua -autoboot_delay 0

Values: 0x02 MX-80/FX-80/JX-80/FX-85/LX-800/LX-810L/AP-2000/EX-800/Gemini-10X,
0x04 RX-80/LX-86.

## trace.lua

Debugger trace of a printer CPU (needs -debug -debugger none); replace SCRATCH
with an output directory and the CPU tag as needed.

## compose.py, units.sh

Composition of the shared files (bus.lua, ctronics.cpp, rs232.cpp, ...) per
printer unit by keyword classification against upstream base b7f483f3a1d;
used to cut the units of the former mame-printers branch (archived as
archive/printers-units-2026-11).

## ROM sets for testing (zip name = device short name)

hplj3p, hd44780, epson_mx80 (+ copy as epson_mx80dp), epson_fx80, epson_jx80,
epson_rx80, epson_fx85, lx86, lx800, lx810l, ap2000, ex800, fx880, ap3250,
gem10x, c200gx, bx80, iwriter2.

## LaserJet IIIP scripts

- menu3.lua: SEQ=OMMMMMMMMMMMEMPEO presses LaserJet keys in turn (O on line,
  M menu, E enter, P plus) and takes a snapshot after each; this sequence
  selects I/O SERIAL
- test2.lua: self test (off line, Alt held, Enter)
- led.lua: prints the On Line indicator output
- macprint.lua: maclc475 with the printer on the printer port: keys for
  SimpleText, Cmd-O, Cmd-P, Return; logs the bytes the printer reads (RXLOG)
- acia6850-break-reset.patch: MC6850 receiver waits for mark after master
  reset (fixes 79 SERVICE with a break during power on)
