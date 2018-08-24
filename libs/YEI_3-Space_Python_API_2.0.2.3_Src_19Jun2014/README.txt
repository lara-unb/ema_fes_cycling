YEI 3-Space™ Python API
=============================
##########
#Overview#
##########
The YEI 3-Space Python Application Programming Interface (API) is a series of
classes, functions, structure definitions, and static variables designed to make
writing applications that utilize the devices of the YEI 3-Space Sensor family
fast and easy to support. Binary libraries and source files are included for
convenient integration into projects of any size or application.

This version of the API is distributed as a collection of Python source files
and a Windows binary application. The API is intended to by utilized by simply
importing the "threespace_api" module. The other included source files are used
by the "threespace_api" module to perform tasks such as managing serial
connections, parsing binary data, and automate Windows-specific portions of the
serial port scanning process.

###################
#File Descriptions#
###################
threespace_api.py - The main module that programmers will be interacting with
    when utilizing the Python API. This module contains all the classes that are
    instanced by the programmer when connecting to 3-Space Sensor devices. The
    comments included with each class and function, document the high level
    behavior of the class or function. For documentation on specific commands
    for a 3-Space Sensor device, please see the user manual for the device.

threespace_utils.py - A utility library that the API uses and for platform
    specific utilities to import.

win32_threespace_utils.py - A Windows specific utility library that the API uses
    for connecting to 3-Space Sensor devices and getting additional information.

try_port folder - This folder contains the files necissary for running the
    contained "try_port.exe" program. This is a program that is run when serial
    ports are reconnected. This program exists as a workaround for a bug in the
    Microsoft CDC driver utilized by the 3-Space Sensor family of hardware.

##############
#Change Notes#
##############

2.0.2.3 - Fixed an issue with uknown ports in the getDeviceInfoFromComPort
    function.

2.0.2.2 - Fixed issue with the TSDongle's f8WriteRead function.

2.0.2.1 - Fixed issue with high range versions of the sensors. Added functions
    for getting and setting the amount of wireless retries the Python API will
    perform.

2.0.2.0 - Fixed issue when a dongle's wireless table uses an abbreviated serial
    number. Upped the read queue to 15. Fixed timeout from never being removed
    from the read queue. Improved the timeout routine and increased timeout of
    commands. Changed Broadcaster retries default to 10. Added a dynamic timeout
    to accommodate a larger queued read/write. Fixed some minor bugs with
    debugging and the micro and water-tight devices. The getStreamingBatch
    function now works.

2.0.1.0 - Added file threespace_utils.py so that the API can be cross-platform.
    Merged files yei_list_ports_fix.py and yei_bluetooth_utils.py with
    win32_threespace_utils.py to condense the code.
    Changed default timestamp from system to the device's timestamp.
    Added new firmware functions setMagnetoresistiveThreshold,
    setAccelerometerResistanceThreshold, offsetWithCurrentOrientation,
    resetBaseOffset, offsetWithQuaternion, setBaseOffsetWithCurrentOrientation,
    getMagnetoresistiveThreshold, getAccelerometerResistanceThreshold, and
    getOffsetOrientationAsQuaternion.

2.0.0.4 - Changed __version_firmware to a tuple of the compatibility versions.
    Changed checkSoftwareVersionFromPort to return compatibility number. Fixed
    getReceptionBitfield to be a short. Changed setInterruptType to setPinMode
    and fixed parameters. Changed getInterruptType to getPinMode. Added
    turnOnMassStorage function. Added turnOffMassStorage function. Added
    checkLongCommands. Added compatibility checks to writes, if function is not
    supported it will throw an exception. Changed setStaticAccelerometerRhoMode
    to setStaticAccelerometerTrustValue. Changed
    setConfidenceAccelerometerRhoMode to setConfidenceAccelerometerTrustValues.
    Changed setStaticCompassRhoMode to setStaticCompassTrustValue. Changed
    setConfidenceCompassRhoMode to setConfidenceCompassTrustValues. Changed
    getAccelerometerRhoValue to getAccelerometerTrustValues and fixed
    parameters. Changed getCompassRhoValue to getCompassTrustValues and fixed
    parameters. Added a retry of 4 times for _wirelessWriteRead.

2.0.0.3 - Added functions broadcastSynchronizationPulse and getReceptionBitfield
    to TSDongle class. Fixed a Python 3 print bug in yei_list_ports_fix.py
    Added more information to the checkSoftwareVersionFromPort function.

2.0.0.2 - Added versioning in the threespace_api.py file. Added functions
    getDefaultCreateDeviceBaudRate, setDefaultCreateDeviceBaudRate, and
    getSensorToDongle. Changed functions getSensorInfo to
    getDeviceInfoFromComPort, getLatestData to getLatestStreamData, and
    addSensor to setSensorToDongle. Added an attribute called 'baudrate' to the
    classes and a default parameter called 'baudrate' for the creation of the
    classes.

2.0.0.1 - Added an __init__.py and updated examples for installer. Fixed an
    error with bytestrings for 3K Python.

2.0.0.0 - Beta Release for using the 2.0 version of firmware on 3-Space Sensor
    devices

0.0.0.10 - Initial Beta Release

#########
#License#
#########
The YEI 3-Space Python API is released under the YEI 3-Space Open Source
License, which allows for both non-commercial use and commercial use with
certain restrictions.

License Overview
----------------
YEI provides two license options for YEI 3-Space Covered Works. Covered Works
include, but are not limited to, programs, project files, source files,
firmware, sample program output, and packaged assets created by YEI Corporation,
and described as being covered under the YEI 3-Space Open Source License.
Software and data considered Covered Works include, but are not limited to,
YEI 3-Space Suite, YEI 3-Space Mocap Studio, YEI 3-Space Blender Motion Capture
plug-in, demonstration programs, and the YEI 3-Space Open Source family of
firmware.

User Works outputted by Covered Works are considered program output, and all
rights and copyright remain in the User. Examples of User Works include motion
capture project files (such as .tsh files), node graph XML files, animation
files (such as .bvh files), sensor data logs (data output from the YEI 3-Space
Suite), and sensor data saved to an SD card (such as from the data logging
sensor).

YEI offers two license options for Covered Works, a Commercial Use License and a
Non- Commercial Use License. Before choosing and accepting the appropriate
license for your needs, please read both options carefully, including the key
Definitions below. If you are unsure of the appropriate licensing based on your
intended use, please contact us directly at (support@yeitechnology.com).

Definitions
-----------
"Compilation"means any work which combines Covered Works with any services,
programs, code or other products not governed by the terms of this License.

"Distribution" and "Distributed" mean the sale, re-sale, licensing, supplying,
redistribution, conveyance, packaging, shipping, copying, or public or private
posting of access to Covered Works.

"Improvement" means a modification to Covered Works that corrects a bug, defect
or error without affecting the overall functionality.

Modification means any alteration, addition to, or deletion from, the substance
or structure of Covered Works including, without limitation, (a) any addition to
or deletion from Covered Works, (b) any derivative of Covered Works, or (c) any
work that contains any part of Covered Works.

"Pecuniary Gain" means any action resulting in or intended to result in a direct
or indirect monetary gain or economic benefit to any person or entity involved
in the use, reproduction or distribution of Covered Works. Pecuniary gain may
be, but is not limited to, a license fee, a user fee, a packaging, distribution,
copying or shipping fee. However, the assessment of support fees, consulting
fees, or other service-related fees by third parties, so long as the fees are
not required as part of the distribution of Covered Works, and are not
incorporated into third party products and/or software, is not considered
Pecuniary Gain for the purposes of this License.

YEI 3-Space™ Non-Commercial Use License
---------------------------------------
For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL
v.3, subject to the YEI 3-Space Open Source Licensing Overview and Definitions.
Non-Commercial Use, for the purposes of this License, prohibits the distribution
of Covered Works for Pecuniary Gain. Non-Commercial Use, for the purposes of
this License, does not permit Covered Works to be incorporated, in whole or in
part, in its original or any revised form, in any other software, application,
product, or service that is subsequently Distributed.

Non-Commercial Use, for the purposes of this License, allows users to copy,
modify and distribute the source code, with one very important exception.
The distribution cannot be done for the purpose of reaping a commercial profit
or gain. As an example, if a user embeds or relies on the Covered Works in a
product that is then sold to a third party either alone or as a package or
component with other products or services, this would be a violation of the
Non-Commercial Use License, even though this type of use would be permitted
under the GNU GPL.

The GNU General Public License (GPL) is available at:
http://www.gnu.org/licenses/gpl.html

YEI 3-Space™ Commercial Use License
-----------------------------------
For Commercial Use, a YEI Commercial/Redistribution License is required,
pursuant to the YEI 3-Space Open Source Licensing Overview and Definitions.
Commercial Use, for the purposes of this License, means the use, reproduction
and/or Distribution, either directly or indirectly, of the Covered Works or any
portion thereof, or a Compilation, Improvement, or Modification, for Pecuniary
Gain. A YEI Commercial/Redistribution License may or may not require payment,
depending upon the intended use. Contact us directly at
support@yeitechnology.com to inquire about or obtain a YEI
Commercial/Redistribution License.

Commercial Use, for the purposes of this License, includes but is not limited
to: (a) integrating Covered Works with other software or hardware for Pecuniary
Gain; (b) the licensing, selling, or Distributing of Covered Works for Pecuniary
Gain; (c) the use of Covered Works, in whole or in part, in any software,
application, product, or service, for Pecuniary Gain (excluding the User Works
that are outputted by Covered Works), or that is provided “free of charge” in
conjunction with, or bundled with, other software, application, products or
services for which a fee or payment is assessed.


############
#Contact Us#
############
YEI Corporation
630 Second Street
Portsmouth, OH 45662
(740) 355-9029
support@yeitechnology.com

©2014 YEI Corporation

rev. 17 Feb 2014
