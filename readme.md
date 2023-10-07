# Arduino driver to program SkyTraq PX100 series receivers

This library is used to send binary messages to SkyTraq PX100 (i.e. PX1125R, PX1122R, etc) receivers.

Initial functionality includes implementation to send a message that the host program defines.

SkyTraq publishes their binary message protocol:
 https://www.navsparkforum.com.tw/download/file.php?id=1162&sid=dc2418f065ec011e1b27cfa77bf22b19
 
 Note that some messages do not seem to be up to date or the latest binary protocol documentation is not easy to find. It is also possible to use SkyTraq's programming tool to generate messages to the receiver and copy the binary (hex) codes that are printed by the program.








