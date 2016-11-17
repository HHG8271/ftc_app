/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * This file contains an minimal example of a Linear Autonomous "OpMode".
 *
 * This particular OpMode just executes a basic Autonomous driving for time, not using encoders
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Don't forget to comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name= "Autonomous", group="Examples")  // @TeleOp(...) is the other common choice

public class Autonomous1 extends LinearOpMode {
    ColorSensor colorSensor;// STATING SENSORS
    ModernRoboticsI2cRangeSensor rangeSensor;
    /* Declare OpMode members. */
    //motors
    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorFlick;

    //servos
    Servo Servo;
    OpticalDistanceSensor odsSensor;

    //Create and set default hand positions variables. To be determined based on your build
    double left = 0.2; // POSITIONS OF BEACON SERVO
    double right = 0.8;

    @Override

    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorFlick = hardwareMap.dcMotor.get("motorFlick");
        Servo = hardwareMap.servo.get("Servo");     //autonomous code example below uses servo for arm instead/in addition to motor
        //assuming a pushBot configuration of two servo grippers


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorFlick.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration

        //Set servo hand grippers to open position.
        Servo.setPosition(.5);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            motorFlick.setPower(1);
            Thread.sleep(4000);
            /*Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            if (rangeSensor.rawUltrasonic() >4.15 ) {// while Ultrasonic is farther than 4.15 away
                // follow the line

                    motorLeft.setPower(.2);// driving straight.
                    motorRight.setPower(.2);
            }
                else if(odsSensor.getLightDetected()<5&&rangeSensor.rawUltrasonic()>4.15)
                {
                    motorLeft.setPower(.2);
                    motorRight.setPower(-.1);

                }

            if (rangeSensor.rawUltrasonic() <4.15 ) {
                motorRight.setPower(0);
                motorLeft.setPower(0);
                if (colorSensor.red() > 20) {
                    Servo.setPosition(right);


                } else if (colorSensor.blue() > 20) {
                    Servo.setPosition(left);

                }
            }

            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Raw",    odsSensor.getRawLightDetected());
            telemetry.addData("Normal", odsSensor.getLightDetected());
            telemetry.update();*/

           idle();
        }


        }
    }

        //runtime.reset();
/*
        /************************
         * Autonomous Code Below://
         *************************/
        /*DriveForwardTime(DRIVE_POWER, 4000);
        TurnLeft(DRIVE_POWER, 1000);
        StopDrivingTime(2000);

        DriveForwardTime(DRIVE_POWER, 4000);
        TurnRight(DRIVE_POWER, 1000);
        StopDrivingTime(2000);

        RaiseArm();
        DriveForwardTime(DRIVE_POWER, 4000);
        StopDriving();

       

    }//runOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    /*double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void StopDriving()
    {
        DriveForward(0);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        DriveForwardTime(0, time);
    }

    public void TurnLeft(double power, long time) throws InterruptedException
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        Thread.sleep(time);
    }

    public void TurnRight(double power, long time) throws InterruptedException
    {
        TurnLeft(-power, time);
    }

    public void RaiseArm()
    {
        armServo.setPosition(.8); //note: uses servo instead of motor.
    }

    public void LowerArm()
    {
        armServo.setPosition(.2);
    }*/


//ExampleAutonomous
