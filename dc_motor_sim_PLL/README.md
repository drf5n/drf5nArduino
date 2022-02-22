# dc_motor_sim_PLL.ino
Simulate a DC permanent magnet motor on an Arduino

This uses a PID to control motor angular position as a Phase Locked Loop, in contrast to
using a PID to control the motor speed.

The PmMotor class is a rough state-space model of a DC permanent Magnet Motor, built along the lines of 
https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling

    // motor simiulated by a state space motor in class PmMotor
    // This aims at PLL control of angular position rather than RPM
    // See https://www.romanblack.com/onesec/DCmotor_xtal.htm

See the wokwi at https://wokwi.com/arduino/projects/324274590349001300

drf5n 2022-02-22


