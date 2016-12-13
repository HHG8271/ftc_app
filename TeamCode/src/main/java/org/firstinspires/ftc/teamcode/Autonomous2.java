package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
/**
 * Created by user on 12/1/2016.
 */
@Autonomous(name= "Autonomous1", group="Examples")
public class Autonomous2 extends LinearOpMode {
    ColorSensor color_sensor;


    DcMotor motorFlick;
    DcMotor motor_left;
    DcMotor motor_right;

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
        color_sensor = hardwareMap.colorSensor.get("color_sensor");

        // Set the LED in the beginning
        color_sensor.enableLed(bLedOn);


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        motorFlick = hardwareMap.dcMotor.get("motorFlick");
        motor_right = hardwareMap.dcMotor.get("motor_right");
        motor_left = hardwareMap.dcMotor.get("motor_left");

        //autonomous code example below uses servo for arm instead/in addition to motor
        //assuming a pushBot configuration of two servo grippers


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery

        motorFlick.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        motor_left.setDirection(DcMotor.Direction.FORWARD);
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        //Set servo hand grippers to open position.


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {


            motorFlick.setPower(1);
            Thread.sleep(1000);

            motorFlick.setPower(0);
            Thread.sleep(1000);

            motorFlick.setPower(1);
            Thread.sleep(1000);

            motorFlick.setPower(0);
            Thread.sleep(1000);

            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Red  ", color_sensor.red());


            if(color_sensor.red()<10 ) {
                motor_right.setPower(.4);
                motor_left.setPower(.4);


            }
           else if( color_sensor.red()>10 )
            {

                motor_right.setPower(0);
                motor_left.setPower(0);

            }
            telemetry.update();
        }


    }
}
