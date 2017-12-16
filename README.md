# UbloxM8Q_GPS

A Particle library for UbloxM8Q_GPS
This library is used by the AssetTracker2 library
At this writing it has not been used except with the AssetTracker2 library

## Usage

Connect AssetTracker2 hardware, add the UbloxM8Q_GPS library and the
AssetTracker2 library to your project. The AssetTracker2 library also uses
Adafruit_LIS3DH library and the Adafruit_Sensor library for the acceleration
detector. See the README for the AssetTracker2 library for more details.

If you are using the AssetTracker2 library it is not necessary 
to #include UblocM80Q_GPS because it is included by that library.

See the [examples](examples) folder for AssetTracker2 for more details.

## Documentation

UbloxM8Q_GPS library by Lynd Wieman
For use with the Particle Electron AssetTracker 2.

When I first used the AssetTracker 2 hardware I tried the AssetTracker
and Adafruit_GPS drivers which were written for a different GPS receiver.
Although they worked to parse the NMEA data that the ublox receiver
generates by default, they used the wrong protocol for configuration and
so all commands were ignored.
Also, to get the most benefit from the ublox receiver the ubx protocol must
be used.
Rather than support two protocols and suffer with the complexities involved
I only implemented the ubx protocol. When the receiver is inialized
the NMEA (ASCII) sentences are turned off and ubx protocol messages are
enabled.

The ublox driver was written to be as compatible as possible with the
AssetTracker driver but the AssetTracker library had dependencies on the 
non-ublox GPS receiver that was used for AssetTracker 1.
I have created the AssetTracker2 library to remove the dependencies, but
the AssetTracker2 is now inextricably depedendent on this ublox driver.

The external interface presented by AssetTracker and AssetTracker2 are 
similar but access methods that were specific to NMEA have been removed.
 
This implementation is based on the ublox spec:
 https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221)_Public.pdf

The version reported by the ublox receiver I am testing with is:
$GNTXT,01,01,02,u-blox AG - www.u-blox.com*4E
$GNTXT,01,01,02,HW UBX-M8030 00080000*60
$GNTXT,01,01,02,ROM CORE 3.01 (107888)*2B
$GNTXT,01,01,02,FWVER=SPG 3.01*46
$GNTXT,01,01,02,PROTVER=18.00*11
$GNTXT,01,01,02,GPS;GLO;GAL;BDS*77
$GNTXT,01,01,02,SBAS;IMES;QZSS*49
$GNTXT,01,01,02,GNSS OTP=GPS;GLO*37
$GNTXT,01,01,02,LLC=FFFFFFFF-FFFFFFED-FFFFFFFF-FFFFFFFF-FFFFFF69*23
$GNTXT,01,01,02,ANTSUPERV=AC SD PDoS SR*3E
$GNTXT,01,01,02,ANTSTATUS=DONTKNOW*2D

The above is generated from the receiver when it is reset. You have to 
have the serial port listening when it is booted which is tricky because
particle CLI takes a little while to connect. I captured the above by
putting a delay(10000) in my startup code after enabling the serial port and
before turning on the receiver.
 
There is a timing problem on startup that is related to trying
to initialize the GPS receiver while the cell, network, and cloud
connections are being initialized by the particle system software.
The symptom is the GPS receiver doesn't get configured correctly and so
the driver never receives ubx messages immediately after initialization.

You can avoid the startup problem by starting in SYSTEM_MODE(MANUAL),
then call Particle.connect() after the GPS receiver is running.

## Contributing

Here's how you can make changes to this library and eventually contribute those changes back.

To get started, [clone the library from GitHub to your local machine](https://help.github.com/articles/cloning-a-repository/).

Change the name of the library in `library.properties` to something different. You can add your name at then end.

Modify the sources in <src> and <examples> with the new behavior.

To compile an example, use `particle compile examples/usage` command in [Particle CLI](https://docs.particle.io/guide/tools-and-features/cli#update-your-device-remotely) or use our [Desktop IDE](https://docs.particle.io/guide/tools-and-features/dev/#compiling-code).

After your changes are done you can upload them with `particle library upload` or `Upload` command in the IDE. This will create a private (only visible by you) library that you can use in other projects. Do `particle library add UbloxM8Q_GPS_myname` to add the library to a project on your machine or add the UbloxM8Q_GPS_myname library to a project on the Web IDE or Desktop IDE.

At this point, you can create a [GitHub pull request](https://help.github.com/articles/about-pull-requests/) with your changes to the original library. 

If you wish to make your library public, use `particle library publish` or `Publish` command.

## LICENSE
Copyright 2017 Lynd Wieman

                   GNU LESSER GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.


  This version of the GNU Lesser General Public License incorporates
the terms and conditions of version 3 of the GNU General Public
License, supplemented by the additional permissions listed below.

  0. Additional Definitions.

  As used herein, "this License" refers to version 3 of the GNU Lesser
General Public License, and the "GNU GPL" refers to version 3 of the GNU
General Public License.

  "The Library" refers to a covered work governed by this License,
other than an Application or a Combined Work as defined below.

  An "Application" is any work that makes use of an interface provided
by the Library, but which is not otherwise based on the Library.
Defining a subclass of a class defined by the Library is deemed a mode
of using an interface provided by the Library.

  A "Combined Work" is a work produced by combining or linking an
Application with the Library.  The particular version of the Library
with which the Combined Work was made is also called the "Linked
Version".

  The "Minimal Corresponding Source" for a Combined Work means the
Corresponding Source for the Combined Work, excluding any source code
for portions of the Combined Work that, considered in isolation, are
based on the Application, and not on the Linked Version.

  The "Corresponding Application Code" for a Combined Work means the
object code and/or source code for the Application, including any data
and utility programs needed for reproducing the Combined Work from the
Application, but excluding the System Libraries of the Combined Work.

  1. Exception to Section 3 of the GNU GPL.

  You may convey a covered work under sections 3 and 4 of this License
without being bound by section 3 of the GNU GPL.

  2. Conveying Modified Versions.

  If you modify a copy of the Library, and, in your modifications, a
facility refers to a function or data to be supplied by an Application
that uses the facility (other than as an argument passed when the
facility is invoked), then you may convey a copy of the modified
version:

   a) under this License, provided that you make a good faith effort to
   ensure that, in the event an Application does not supply the
   function or data, the facility still operates, and performs
   whatever part of its purpose remains meaningful, or

   b) under the GNU GPL, with none of the additional permissions of
   this License applicable to that copy.

  3. Object Code Incorporating Material from Library Header Files.

  The object code form of an Application may incorporate material from
a header file that is part of the Library.  You may convey such object
code under terms of your choice, provided that, if the incorporated
material is not limited to numerical parameters, data structure
layouts and accessors, or small macros, inline functions and templates
(ten or fewer lines in length), you do both of the following:

   a) Give prominent notice with each copy of the object code that the
   Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the object code with a copy of the GNU GPL and this license
   document.

  4. Combined Works.

  You may convey a Combined Work under terms of your choice that,
taken together, effectively do not restrict modification of the
portions of the Library contained in the Combined Work and reverse
engineering for debugging such modifications, if you also do each of
the following:

   a) Give prominent notice with each copy of the Combined Work that
   the Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the Combined Work with a copy of the GNU GPL and this license
   document.

   c) For a Combined Work that displays copyright notices during
   execution, include the copyright notice for the Library among
   these notices, as well as a reference directing the user to the
   copies of the GNU GPL and this license document.

   d) Do one of the following:

       0) Convey the Minimal Corresponding Source under the terms of this
       License, and the Corresponding Application Code in a form
       suitable for, and under terms that permit, the user to
       recombine or relink the Application with a modified version of
       the Linked Version to produce a modified Combined Work, in the
       manner specified by section 6 of the GNU GPL for conveying
       Corresponding Source.

       1) Use a suitable shared library mechanism for linking with the
       Library.  A suitable mechanism is one that (a) uses at run time
       a copy of the Library already present on the user's computer
       system, and (b) will operate properly with a modified version
       of the Library that is interface-compatible with the Linked
       Version.

   e) Provide Installation Information, but only if you would otherwise
   be required to provide such information under section 6 of the
   GNU GPL, and only to the extent that such information is
   necessary to install and execute a modified version of the
   Combined Work produced by recombining or relinking the
   Application with a modified version of the Linked Version. (If
   you use option 4d0, the Installation Information must accompany
   the Minimal Corresponding Source and Corresponding Application
   Code. If you use option 4d1, you must provide the Installation
   Information in the manner specified by section 6 of the GNU GPL
   for conveying Corresponding Source.)

  5. Combined Libraries.

  You may place library facilities that are a work based on the
Library side by side in a single library together with other library
facilities that are not Applications and are not covered by this
License, and convey such a combined library under terms of your
choice, if you do both of the following:

   a) Accompany the combined library with a copy of the same work based
   on the Library, uncombined with any other library facilities,
   conveyed under the terms of this License.

   b) Give prominent notice with the combined library that part of it
   is a work based on the Library, and explaining where to find the
   accompanying uncombined form of the same work.

  6. Revised Versions of the GNU Lesser General Public License.

  The Free Software Foundation may publish revised and/or new versions
of the GNU Lesser General Public License from time to time. Such new
versions will be similar in spirit to the present version, but may
differ in detail to address new problems or concerns.

  Each version is given a distinguishing version number. If the
Library as you received it specifies that a certain numbered version
of the GNU Lesser General Public License "or any later version"
applies to it, you have the option of following the terms and
conditions either of that published version or of any later version
published by the Free Software Foundation. If the Library as you
received it does not specify a version number of the GNU Lesser
General Public License, you may choose any version of the GNU Lesser
General Public License ever published by the Free Software Foundation.

  If the Library as you received it specifies that a proxy can decide
whether future versions of the GNU Lesser General Public License shall
apply, that proxy's public statement of acceptance of any version is
permanent authorization for you to choose that version for the
Library.
