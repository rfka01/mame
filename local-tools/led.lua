local last = -1
sub = emu.add_machine_frame_notifier(function()
  local v = manager.machine.output:get_value("hplj3p_online_led")
  if v ~= last then print(string.format("%.3f LED %d", manager.machine.time:as_double(), v)); last = v end
end)
