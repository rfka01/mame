C=src/devices/bus/centronics
L=src/emu/layout
U7="$C/epson_mx80.cpp $C/epson_mx80.h $C/epson_fx80.cpp $C/epson_fx80.h $C/epson_rx80.cpp $C/epson_rx80.h $L/epson_mx80.lay $L/epson_fx80.lay $L/epson_jx80.lay $L/epson_fx85.lay $L/epson_rx80.lay"
SHARED="scripts/src/bus.lua scripts/src/machine.lua $C/ctronics.cpp src/devices/bus/rs232/rs232.cpp src/mame/mame.lst src/emu/xtal.cpp"
apply_whole() { for f in "$@"; do mkdir -p $(dirname $f); git show origin/master:$f > $f; done; }
apply_shared() { python3 -c "
import sys; sys.path.insert(0,'$S'); import compose
for p in '$SHARED'.split():
    open(p,'w').write(compose.compose(eval('$1'),p))"; }
