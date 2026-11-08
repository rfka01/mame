local val = tonumber(os.getenv("KEYVAL"))
local stage = 0
local cfg, pw
for tag, port in pairs(manager.machine.ioport.ports) do
  for name, f in pairs(port.fields) do
    if name:find("held at power on") then cfg = f end
    if name:find("Power switch") then pw = f end
  end
end
print("cfg", cfg ~= nil, "pw", pw ~= nil)
sub = emu.add_machine_frame_notifier(function()
  local t = manager.machine.time:as_double()
  if stage == 0 and t > 2 then cfg.user_value = val; pw:set_value(1); stage = 1
  elseif stage == 1 and t > 2.2 then pw:clear_value(); stage = 2 end
end)
