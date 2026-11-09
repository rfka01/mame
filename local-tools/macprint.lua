local f = {}
for tag, port in pairs(manager.machine.ioport.ports) do
  if tag:find("hle_keyboard") then for name, fld in pairs(port.fields) do f[name] = fld end end
end
local key = { u = "u  U", o = "o  O", p = "p  P", CMD = "Command / Open Apple", RET = "Return" }
-- steps: {time, list of keys to hold}
local steps = {
  {92, {"RET"}}, {100, {"u"}}, {103, {"CMD", "o"}}, {130, {"CMD", "p"}}, {150, {"RET"}},
}
local cpu = manager.machine.devices[":printer:hplj3p:printer:maincpu"]
local sp = cpu.spaces["program"]
local out = io.open(os.getenv("RXLOG"), "wb")
local lastst = -1
tr = sp:install_read_tap(0x300000, 0x300003, "acia", function(offset, data, mask)
  local t = manager.machine.time:as_double()
  if offset == 0x300002 then out:write(string.char(data & 0xff)); out:flush()
  else
    local st = data & 0xff
    if st & 0x70 ~= 0 and st ~= lastst then print(string.format("%.3f ACIA status %02x", t, st)) end
    lastst = st
  end
end)
local i, held, rel = 1, nil, 0
local snaps = {}
for t in string.gmatch(os.getenv("SNAPS") or "", "[^,]+") do table.insert(snaps, tonumber(t)) end
local si = 1
sub = emu.add_machine_frame_notifier(function()
  local t = manager.machine.time:as_double()
  if held and t > rel then for _, k in ipairs(held) do f[key[k]]:clear_value() end held = nil end
  local s = steps[i]
  if not held and s and t > s[1] then
    for _, k in ipairs(s[2]) do f[key[k]]:set_value(1) end
    held = s[2]; rel = t + 0.25; i = i + 1
    print(string.format("%.1f keys %s", t, table.concat(s[2], "+")))
  end
  if snaps[si] and t > snaps[si] then manager.machine.video:snapshot(); si = si + 1 end
end)
