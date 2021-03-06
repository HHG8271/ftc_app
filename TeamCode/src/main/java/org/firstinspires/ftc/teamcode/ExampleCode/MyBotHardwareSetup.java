
package org.firstinspires.ftc.teamcode.ExampleCode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbServoController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


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

public class MyBotHardwareSetup  {

   /* Declare Public OpMode members.
    *these are the null statements to make sure nothing is stored in the variables.
    */

    //motors
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;
    public DcMotor motorArm = null;

    //servos
    public Servo servoHandL = null;
    public Servo servoHandR = null;

    //sensors
    public GyroSensor gyro  = null;

    /* local OpMode members. */
    HardwareMap hwMap        = null;

    //Create and set desired variables, i.e. default hand positions. To be determined based on your build
    final static double CLOSED = 0.2;
    final static double OPEN = 0.8;
    final static double MOTOR_STOP = 0.0; // sets motor power to zero

   /* Constructor   // this is not required as JAVA does it for you, but useful if you want to add
    * function to this method when called in OpModes.
    */
    public  MyBotHardwareSetup() {
    }

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /************************************************************
         * MOTOR SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        motorLeft = hwMap.dcMotor.get("motorL");
        motorRight = hwMap.dcMotor.get("motorR");
        motorArm = hwMap.dcMotor.get("motorArm");

        // Set the drive motor directions:
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorArm.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

        //Keep the motors from moving during initialize.
        motorLeft.setPower(MOTOR_STOP);
        motorRight.setPower(MOTOR_STOP);
        motorArm.setPower(MOTOR_STOP);

        // Set motors to run USING or WITHOUT encoders
        // Depending upon your configuration and use
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /************************************************************
         * SERVO SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        servoHandL = hwMap.servo.get("servoHandL");
        servoHandR = hwMap.servo.get("servoHandR");

        //Set servo hand grippers to open position.
        servoHandL.setPosition(OPEN);
        servoHandR.setPosition(OPEN);

        /************************************************************
         * SENSOR SECTION
         ************************************************************/
        //Define sensors
        gyro = hwMap.gyroSensor.get("gyro");
   }

}

