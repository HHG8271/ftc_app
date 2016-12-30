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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear Tele "OpMode".
 *
 * This particular OpMode just executes a basic Tank Drive, Arm and 2 Servos for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Example: TeleOp", group="Examples")  // @Autonomous(...) is the other common choice
//@Disabled
public class HardwareExample_TeleOp extends LinearOpMode {

    MyBotHardwareSetup robot = new MyBotHardwareSetup(); //set up remote to robot hardware configuration

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap); //initializes hardware

        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /************************
         * TeleOp Code Below://
         *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)

            telemetry.addLine(); //need to finish format
            telemetry.update();

            // tank drive set to gamepad1 joysticks
            //(note: The joystick goes negative when pushed forwards)
            robot.motorLeft.setPower(gamepad1.left_stick_y);
            robot.motorRight.setPower(gamepad1.right_stick_y);

            // Arm Control - Uses dual buttons to control motor direction
            if(gamepad1.right_bumper)
            {
                robot.motorArm.setPower(-gamepad1.right_trigger); // if both Bumper + Trigger, then negative power, runs arm down
            }
            else
            {
                robot.motorArm.setPower(gamepad1.right_trigger);  // else trigger positive value, runs arm up
            }

            //servo commands
            if(gamepad1.a) //button 'a' will open
            {
                robot.servoHandR.setPosition(robot.OPEN);
                robot.servoHandL.setPosition(robot.OPEN);
            }
            else if (gamepad1.b) //button 'b' will close
            {
                robot.servoHandR.setPosition(robot.CLOSED);
                robot.servoHandL.setPosition(robot.CLOSED);
            }


            idle(); // Allows other parallel processes to run
        }
    }
}
