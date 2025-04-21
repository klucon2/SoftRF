# SoftRF (CK's version)


## Preface

This is a fork of Moshe Braners's SoftRF fork, wich original comes from Linar Yusupov.
I originally bought the T-Echo to use it as a separate flarm device. Since my current vario (BluFly Vario) has some charging issues, I though to myself why not just use the BMP280 sensor in the T-Echo to get my vario readings and combine it with FlySkyHy, which takes care of the audio part. SoftRF in its standard version by Lyusupov unfortunately does not provide the pressure readings often enough to utilize it as a vario. Moshe's version fixed that a bit, but I still wanted a cleaner and faster reading. So I started on adjusting SoftRF so it would fit my needs. During the course of the adjustments I though to myself why not also use the screen to present some usefull information during the flight. So I also created a new "VarioView". The idea is that in a worst case scenario I can use the T-Echo without a smartphone.

Currently I have implemented teh following changes to Moshe's version:

- adjusted oversampling rate of the BMP280 sensor
- activated internal IIR Filter of the BMP280 sensor
- introduced a Kalmanfilter for the sensor's barometic pressure output
- added a second dampening Kalmanfilter to the barometic pressure to receive an almost constant pressure reading when the system is on the same hight, but still be reactive enough for when the pressure/altitude of the system changes (ergo usable as a vario).
- added a new "VarioView" dispalying the current altitude [m] at the top, plotting the altitude of the last 100 seconds below in a graph, displaying the current speed over ground [km/h] and showing the wind direction relative to the heading.
- I deactivated the "Status View" and the "Time View" as these info are not relevant for me, but increase the time to scroll through all the views

<p><img src="https://github.com/klucon2/SoftRF/blob/master/software/firmware/documentation/VarioView.JPG"></p>

------------------------------
Original text:

DIY, multifunctional, compatible, sub-1 GHz ISM band radio based proximity awareness system for general aviation.
<br>
[User Guide](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/SoftRF_MB_user_guide.txt)
<br>

<p><img src="https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/T-Beam_MB149_.jpg"></p>

### Latest additions:

* vMB157: revert approx math, optimize TinyGPS, bug fixes
* vMB156: GNSS baud rate autodetection, option to log NMEA to SD card
* vMB155: collect statistics on reception range by relative direction 
* vMB153: record compressed flight logs in flash memory (T-Beam & T-Echo)
* vMB152: settings stored in an editable text file
* vMB149: handles Mode-S in addition to ADS-B
* vMB140: supports adding an SD card and writing flight logs on the T-Beam
* vMB138: supports using add-on GNSS modules, on the T-Beam
* vMB130: supports the GNS5892R ADS-B receiver module, on the T-Beam
* vMB120: supports the latest 2024 radio protocol
* vMB114: import traffic data in GDL90 format (from ADS-B receiver)
* vMB110: added second serial port and data bridging (only on T-Beam)

### Beyond Lyusupov's version:

* Collision prediction for circling aircraft
* Can set aircraft ID for self, ID to ignore, and ID to follow
* Support 3-level collision alarms via buzzer
* Can configure two simultaneous NMEA output destinations
* Settable baud rate for serial output
* Estimates wind while circling, uses for collision prediction
* Corrected frequency hopping and time slots
* Airborne devices can relay radio packets from landed aircraft
* Can adjust SoftRF settings within T-Echo (without an app)

### And on the T-Beam specifically: 

* Louder buzzer via 2-pin differential drive, or external
* Collision-danger traffic VOICE warnings!
* Includes strobe-control logic
* Option to connect to ambient WiFi network instead of creating one
* Option to send data as TCP client instead of TCP server
* Specify server's IP address for TCP client, and choice of 2 ports
* Ability to use WiFi TCP & UDP NMEA outputs simultaneously
* These WiFi options allow wireless output to XCvario
* Detect and use OLED display on either I2C port
* Winch mode ("aircraft type") with variable reported altitude

### What is here:

Source code, and compiled binaries for [ESP32](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SoftRF) and [nRF52](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage) (only).
<br>
<br>

[Documentation files](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/documentation).
<br>
<br>

Modified version of [SkyView](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyView)
<br>
[SkyStrobe](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyStrobe) - a controller for a visibility strobe (and more)
<br>
<br>

For discussions join the [SoftRF Community](https://groups.google.com/g/softrf_community).
<br>
<br>

For additional info see also [Lyusupov's repository](https://github.com/lyusupov/SoftRF).


