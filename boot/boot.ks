@LAZYGLOBAL OFF.

IF NOT EXISTS("1:/boot_init.ks") { RUNPATH("0:/boot_init_select.ks"). }
RUNONCEPATH("1:/boot_init.ks").

pOut("boot.ks v0.1.0 20210715").

RUNONCEPATH(loadScript("lib_launch.ks")).


doLaunch(0, 85, 0).