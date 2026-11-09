local f = {}
for tag, port in pairs(manager.machine.ioport.ports) do
  for name, fld in pairs(port.fields) do f[name] = fld end
end
local O, A, E = f["LaserJet On Line / Reset"], f["LaserJet Alt"], f["LaserJet Enter / Test"]
local st = 0
sub = emu.add_machine_frame_notifier(function()
  local t = manager.machine.time:as_double()
  if st == 0 and t > 32 then O:set_value(1); st = 1
  elseif st == 1 and t > 32.3 then O:clear_value(); st = 2
  elseif st == 2 and t > 34 then A:set_value(1); st = 3
  elseif st == 3 and t > 34.3 then E:set_value(1); st = 4
  elseif st == 4 and t > 34.7 then E:clear_value(); st = 5
  elseif st == 5 and t > 35.0 then A:clear_value(); st = 6 end
end)
