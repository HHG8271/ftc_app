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
    ColorSensor color_mid;
    ModernRoboticsI2cRangeSensor RangeSensor;
    DcMotor motorFlick;
    DcMotor motor_left;
    DcMotor motor_right;
    Servo servo;
    boolean seap=false;
    boolean trap=false;
    boolean far= false;
    double Left =.9;
    double Right=.3;

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
        color_mid =hardwareMap.colorSensor.get("color_mid");
        color_sensor = hardwareMap.colorSensor.get("color_sensor");
        RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        servo = hardwareMap.servo.get("servo");
        // Set the LED in the beginning
        color_sensor.enableLed(bLedOn);
        color_mid.enableLed(bLedOn);

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
        motor_left.setDirection(DcMotor.Direction.REVERSE);
        motor_right.setDirection(DcMotor.Direction.FORWARD);
        //Set servo hand grippers to open position.
        boolean val= false;
        boolean sar= false;
        boolean but= false;
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

            motor_left.setPower(.4);
            motor_right.setPower(-.4);
            Thread.sleep(500);

             val=true;


        while(val==true ){
            if(color_sensor.red()<10 ) {
                motor_right.setPower(.3);
                motor_left.setPower(.3);
                telemetry.addData("Red",color_sensor.red());
                telemetry.update();

            }
            if(color_sensor.red()>10) {
                motor_right.setPower(0);
                motor_left.setPower(0);
                Thread.sleep(800);
                val=false;
                sar=true;
            }

            telemetry.addData("Red  ", color_sensor.red());
            telemetry.update();
        }
            while(sar=true) {


                if(color_sensor.red()>10){

                    motor_left.setPower(.3);
                    motor_right.setPower(.3);
                }
                if(color_sensor.red()<10)
                {
                    motor_left.setPower(0);
                    motor_right.setPower(.3);

                }
                else if(RangeSensor.getDistance(DistanceUnit.CM)<10){
                    motor_left.setPower(0);
                    motor_right.setPower(0);
                    sar=false;
                    far = true;
                }
                telemetry.addData("cm optical", "%.2f. cm", RangeSensor.cmOptical());
                telemetry.addData("cm", "%.2f cm", RangeSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Red  ", color_sensor.red());
                telemetry.update();



                while(far=true)
                {
                   if(color_mid.red()>=6)
                   {
                    servo.setPosition(Left);
                       far=false;
                   }
                    else if(color_mid.red()<6)
                   {
                       servo.setPosition(Right);
                       far=false;

                   }

                    telemetry.addData("Red  ", color_sensor.red());
                    telemetry.update();

                }
                motor_left.setPower(0);
                motor_right.setPower(0);
                Thread.sleep(500);

                motor_right.setPower(-.5);
                motor_left.setPower(-.5);
                Thread.sleep(1000);

                motor_left.setPower(0);
                motor_right.setPower(0);
                Thread.sleep(500);

                motor_left.setPower(-.4);
                motor_right.setPower(.4);
                Thread.sleep(800);

                motor_left.setPower(0);
                motor_right.setPower(0);
                Thread.sleep(500);
                but =true;

                while(but=true)
                {

                    if(color_sensor.red()<10 ) {
                        motor_right.setPower(.3);
                        motor_left.setPower(.3);
                        telemetry.addData("Red",color_sensor.red());
                        telemetry.update();

                    }
                    if(color_sensor.red()>10) {
                        motor_right.setPower(0);
                        motor_left.setPower(0);
                        Thread.sleep(800);
                        but = false;
                        seap = true;
                    }




                }
                while(seap=true)
                {
                    if(color_sensor.red()>10){

                        motor_left.setPower(.3);
                        motor_right.setPower(.3);
                    }
                    if(color_sensor.red()<10)
                    {
                        motor_left.setPower(0);
                        motor_right.setPower(.3);

                    }
                    else if(RangeSensor.getDistance(DistanceUnit.CM)<10){
                        motor_left.setPower(0);
                        motor_right.setPower(0);
                        seap=false;
                        trap = true;
                    }
                    while(trap = true)
                    {
                        if(color_mid.red()>=6)
                        {
                            servo.setPosition(Left);
                            trap=false;
                        }
                        else if(color_mid.red()<6)
                        {
                            servo.setPosition(Right);
                            trap=false;

                        }
                    }
                }

            }

            telemetry.addData("cm", "%.2f cm", RangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Red  ", color_sensor.red());
            telemetry.update();
            idle();
        }


    }
}
