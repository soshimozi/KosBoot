@LAZYGLOBAL OFF.

IF NOT EXISTS("1:/init.ks") { RUNPATH("0:/init_select.ks"). }
RUNONCEPATH("1:/init.ks").

FOR f IN LIST(
  "lib_runmode.ks"
) { RUNONCEPATH(loadScript(f)). }


pOut("Resetting runMode to 0").

runMode(0).