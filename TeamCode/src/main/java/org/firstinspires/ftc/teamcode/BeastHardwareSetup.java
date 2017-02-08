
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by TeameurekaRobotics on 12/30/2016
 *
 * This file contains an example Hardware Setup Class.
 *
 * It can be customized to match the configuration of your Bot by adding/removing hardware, and then used to instantiate
 * your bot hardware configuration in all your OpModes. This will clean up OpMode code by putting all
 * the configuration here, needing only a single instantiation inside your OpModes and avoid having to change configuration
 * in all OpModes when hardware is changed on robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *
 */

public class BeastHardwareSetup {

   /* Declare Public OpMode members.
    *these are the null statements to make sure nothing is stored in the variables.
    */

    //motors
    public DcMotor motor_left = null;

    public DcMotor motor_right = null;
    public DcMotor motorConveyor = null;
    public DcMotor motorFlick = null;
    public DcMotor SpinnyR = null;
    public DcMotor SpinnyL = null;

    //servos
 public Servo servo = null;
  //  public Servo servoHandR = null;

    //sensors
   // public GyroSensor gyro  = null;
    ColorSensor color_sensor;
    ColorSensor color_mid;
    ModernRoboticsI2cRangeSensor RangeSensor;

    /* local OpMode members. */
    HardwareMap hwMap        = null;

    //Create and set desired variables, i.e. default hand positions. To be determined based on your build
    final static double Right = 0.3;
    final static double Left= 0.9;
    final static double MOTOR_STOP = 0.0; // sets motor power to zero

   /* Constructor   // this is not required as JAVA does it for you, but useful if you want to add
    * function to this method when called in OpModes.
    */
    public BeastHardwareSetup() {
    }

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /************************************************************
         * MOTOR SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        motor_left = hwMap.dcMotor.get("motor_left");
        motor_right= hwMap.dcMotor.get("motor_right");
        motorConveyor = hwMap.dcMotor.get("motorConveyor");
        motorFlick = hwMap.dcMotor.get("motorFlick");
        color_mid =hwMap.colorSensor.get("color_sensor");
        color_sensor = hwMap.colorSensor.get("color_mid");
        RangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");


        // Set the drive motor directions:
        motor_left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorFlick.setDirection(DcMotor.Direction.FORWARD);
        motorConveyor.setDirection(DcMotor.Direction.FORWARD);
        // Can change based on motor configuration

        //Keep the motors from moving during initialize.
        motor_left.setPower(MOTOR_STOP);
        motor_right.setPower(MOTOR_STOP);
        motorFlick.setPower(MOTOR_STOP);

        // Set motors to run USING or WITHOUT encoders
        // Depending upon your configuration and use


        /************************************************************
         * SERVO SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        servo = hwMap.servo.get("Servo");
       // servoHandR = hwMap.servo.get("servoHandR");

        //Set servo hand grippers to open position.
       // servoHandL.setPosition(OPEN);
       // servoHandR.setPosition(OPEN);

        /************************************************************
         * SENSOR SECTION
         ************************************************************/
        //Define sensors

   }

}

