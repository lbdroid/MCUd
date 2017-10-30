# MCUd
<br>
Right now, we have a bunch of different Java/Android projects, implementing communications with hardware<br>
through questionable jni libraries. These projects are for the purpose of gaining control over the<br>
unusual hardware design of SYU/FYT car radios, rather than relying on the horribly buggy and untrustworthy<br>
software that is shipped with them, which among other things, also includes spyware.<br>
<br>
These libraries in particular, include;<br>
https://github.com/lbdroid/MCUController<br>
https://github.com/lbdroid/MCUKeyReceiver<br>
https://github.com/lbdroid/BD37033Controller<br>
<br>
MCUController is responsible for passing commands back and forth with the MCU via /dev/ttyS0. The MCU has<br>
some I/O lines (ACC -din, headlights -din, reverse -din, parking brake -din, antenna power -dout, amplifier<br>
power -dout, steering wheel 1 -ain, steering wheel 2 -ain), and the AMFM radio via i2c. Critically, it<br>
sends a particular command sequence when it is preparing to power down several seconds after ignition off.<br>
It also monitors whether or not the application processor is still responsive based on whether or not it<br>
continues to receive heartbeat signals.<br>
<br>
MCUKeyReceiver receives signals from MCUController when steering wheel buttons are pressed. It then<br>
responds to those signals.<br>
<br>
BD37033Controller is responsible for passing commands back and forth with the bd37033 sound processor<br>
via /dev/i2c-4. This sound processor has several stereo inputs and dual stereo + low frequency output.<br>
It controls audio selection, balance, fade, volume, and mixing.<br>
<br>
There are also some utilities;<br>
https://github.com/lbdroid/MCURadio<br>
https://github.com/lbdroid/HFPClient<br>
<br>
HFPClient is a more or less "correct" implementation of an HFP client. It is forked from codeaurora and<br>
modified to implement an incoming call notification, and to be buildable by the SDK rather than only as<br>
part of a full AOSP tree. However, it is doing an ugly job of duplicating functionality that is included<br>
as part of AOSP 8.0.<br>
<br>
MCURadio is a complete cludge that is communicating with the MCUController to send commands to the AMFM<br>
radio hardware.<br>
<br>
Most of the functions of these programs should be handled by Android HALs.<br>
In particular, the audio HAL, radio HAL, power HAL, and car HAL.<br>
But the problem is that the different interfaces we have available to us need to both tie in to multiple<br>
HALs, AND each handle multiple HALs.<br>
<br>
And therefore we require a way to combine *and* divide the resources such that they are distributed<br>
properly to the different HALs.<br>
<br>
That is what MCUd is.<br>
<br>
MCUd will implement several threads for reading data from several different HALs, and several different<br>
interfaces to the car radio. As well the main thread will be responsible for heartbeat and maintenance.<br>
<br>
When MCUd receives a message from any hardware or HAL, it will determine the appropriate destination<br>
for that message and pass it along.<br>
