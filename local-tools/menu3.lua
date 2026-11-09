local f = {}
for tag, port in pairs(manager.machine.ioport.ports) do
  for name, fld in pairs(port.fields) do f[name] = fld end
end
local K = { O = f["LaserJet On Line / Reset"], M = f["LaserJet Menu / Print Fonts"], P = f["LaserJet + / -"], E = f["LaserJet Enter / Test"] }
local seq = os.getenv("SEQ")
local i, state, tnext, cur = 1, "wait", 32, nil
sub = emu.add_machine_frame_notifier(function()
  local now = manager.machine.time:as_double()
  if now < tnext then return end
  if state == "wait" then
    local c = seq:sub(i, i)
    if c == "" then return end
    cur = K[c]; cur:set_value(1); state = "down"; tnext = now + 0.3
  elseif state == "down" then
    cur:clear_value(); state = "snap"; tnext = now + 1.2
  elseif state == "snap" then
    manager.machine.video:snapshot(); i = i + 1; state = "wait"; tnext = now + 0.1
  end
end)
