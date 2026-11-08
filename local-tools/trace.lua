local d = manager.machine.debugger
d:command("trace SCRATCH/tr2.log,:centronics:mx80:maincpu,noloop")
d:command("bpset C3A,1,{tracelog \"RX=%02X\\n\",a; g}")
d.execution_state = "run"
