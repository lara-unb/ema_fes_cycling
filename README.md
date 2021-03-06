# ema_fes_cycling
This repository provides the main code to run the Project EMA FES cycling application. For more info:

* [Introduction](https://github.com/lara-unb/ema_fes_cycling/wiki)
* [Installing on PC](https://github.com/lara-unb/ema_fes_cycling/wiki/1.-Installing-on-PC)
* [Installing on Raspberry Pi](https://github.com/lara-unb/ema_fes_cycling/wiki/1.-Installing-on-RASP)
* [Getting Started on PC](https://github.com/lara-unb/ema_fes_cycling/wiki/2.-Getting-Started-on-PC)
* [Getting Started on Raspberry Pi](https://github.com/lara-unb/ema_fes_cycling/wiki/2.-Getting-Started-on-RASP)
* [Website](http://projectema.com/)

## Updates

The _'master'_ branch is only for PC stable versions, usually followed by a release. Modifications, updates and new features stay on _'develop'_ until tested and approved to go to _'master'_. The changes are first made for PC and then adapted for the embedded system on _'raspema'_. The _'tests'_ branch is rarely used and is for tests not related to a new feature.

## Quick Reference

### Packages:

* https://github.com/lara-unb/ema_common_msgs
* https://github.com/lara-unb/hasomed_rehastim_stimulator
* https://github.com/lara-unb/yostlabs_3space_imu

### Commands:

**`roslaunch ema_fes_cycling trike.launch`**

**`rqt`**

### Structure:

- **cfg:** contains dynamic reconfiguration files

- **config:** keeps initial configuration files

- **launch:** includes ROS launch files

- **scripts:** contains ROS node files and auxiliary modules

- **unused:** out-of-date scripts, legacy code once used for testing or miscellaneous files in general
