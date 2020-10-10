# ema_fes_cycling
This repository provides the main code to run the Project EMA FES cycling application. For more info:

* [Introduction](https://github.com/lara-unb/ema_fes_cycling/wiki)
* [Installing on PC](https://github.com/lara-unb/ema_fes_cycling/wiki/1.-Installing-on-PC)
* [Installing on Raspberry Pi](https://github.com/lara-unb/ema_fes_cycling/wiki/1.-Installing-on-RASP)
* [Getting Started on PC](https://github.com/lara-unb/ema_fes_cycling/wiki/2.-Getting-Started-on-PC)
* [Getting Started on Raspberry Pi](https://github.com/lara-unb/ema_fes_cycling/wiki/2.-Getting-Started-on-RASP)
* [Website](http://projectema.com/)

## Quick Reference

### Packages:

* https://github.com/lara-unb/ema_common_msgs
* https://github.com/lara-unb/hasomed_rehastim_stimulator
* https://github.com/lara-unb/yostlabs_3space_imu

### Commands:

**`roslaunch ema_fes_cycling trike.launch`**

**`roslaunch ema_fes_cycling demo.launch`**

### Structure:

- **config:** keeps initial configuration files

- **launch:** includes ROS launch files

- **other:** out-of-date scripts, legacy code once used for testing or miscellaneous 

- **scripts:** contains ROS node files and auxiliary modules
