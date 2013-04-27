# This project is deprecated

This project is no longer maintained as I will instead focus on a Balancing robot kit, the Balanduino. For more information see the new repository: <https://github.com/TKJElectronics/Balanduino> and the Kickstarter campaign: <http://www.kickstarter.com/projects/tkjelectronics/balanduino-balancing-robot-kit>.

<br>

The code is released under the GNU General Public License.
Developed by Kristian Lauszus

This is the code for my balancing robot/segway. It's made for the mbed board, but can easily be ported to other microcontrollers like the Arduino - see the Arduino version of the code: <https://github.com/TKJElectronics/BalancingRobotArduino>.

The code is also published on the mbed site: <http://mbed.org/users/Lauszus/programs/BalancingRobotPS3>.

I use a [6DOF IMU from Sparkfun](http://www.sparkfun.com/products/10010), I only use 1 of the gyro axis though.
For more info about calculating the pitch and the Kalman filter see my post at the Arduino forum: <http://arduino.cc/forum/index.php/topic,58048.0.html>.

To steer the robot, I use an Arduino with a [USB Host Shield](http://www.circuitsathome.com/products-page/arduino-shields/usb-host-shield-2-0-for-arduino/) on top together with my [PS3 Controller Bluetooth Library](https://github.com/felis/USB_Host_Shield_2.0/blob/master/PS3BT.cpp) for Arduino. The code for the remote can be found at my other repository: <https://github.com/TKJElectronics/BalancingRobotRemote>.

For information about the hardware, see the wiki: <https://github.com/TKJElectronics/BalancingRobot/wiki/Hardware>.

For more information see my blog post at <http://blog.tkjelectronics.dk/2012/03/the-balancing-robot/> or send me an email at: <kristianl@tkjelectronics.dk>.

Also check out the Youtube video of it in action: <http://www.youtube.com/watch?v=N28C_JqVhGU>.