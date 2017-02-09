package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BeastHardwareSetup;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;
/**
 * Created by user on 1/24/2017.
 */
@Autonomous(name = "BeastAuto", group = "Test")
public class BeastAuto extends LinearOpMode {
    public DcMotor motor_left = null;
    public DcMotor motor_right = null;
    public DcMotor motorConveyor = null;
    public DcMotor motorFlick = null;
    public DcMotor SpinnyR = null;
    public DcMotor SpinnyL = null;
    public DcMotor Press = null;

    //servos
    public Servo servo = null;
    //  public Servo servoHandR = null;

    //sensors
    // public GyroSensor gyro  = null;
    public ColorSensor color_sensor;
    public OpticalDistanceSensor ODSRR;
    public OpticalDistanceSensor ODSRF;
    public OpticalDistanceSensor ODSLF;
    public OpticalDistanceSensor ODSLR;



    //Create and set desired variables, i.e. default hand positions. To be determined based on your build
    final static double MOTOR_STOP = 0.0; // sets motor power to zero

    //light sensor variables
    static final double WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;  // adjust to desired motor speed ( 0.0 - 1.0 )
    //encoder variables
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder is 1440, AndyMark NevaRest encoders is 1120
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP >1.0 if geared DOWN. If no gearing set to 1.0
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;


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
        boolean bLedOn = true;
        color_sensor = hardwareMap.colorSensor.get("color_sensor");
        color_sensor.enableLed(bLedOn);
        Press= hardwareMap.dcMotor.get("Press");
        motorFlick = hardwareMap.dcMotor.get("motorFlick");
        motor_right = hardwareMap.dcMotor.get("motor_right");
        motor_left = hardwareMap.dcMotor.get("motor_left");
        ODSRR = hardwareMap.opticalDistanceSensor.get("RR");
        ODSRF = hardwareMap.opticalDistanceSensor.get("RF");
        ODSLF = hardwareMap.opticalDistanceSensor.get("LF");
        ODSLR = hardwareMap.opticalDistanceSensor.get("LR");

        // "Reverse" the motor that runs backwards when connected directly to the battery

        motorFlick.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        motor_left.setDirection(DcMotor.Direction.FORWARD);
        motor_right.setDirection(DcMotor.Direction.REVERSE);


///////////////////////////
        //BEGIN
///////////////////////////
        waitForStart();

        motorFlick.setPower(1);   // FIRE!!!
        Thread.sleep(1000);

        motorFlick.setPower(MOTOR_STOP);  //pause to load next particle
        Thread.sleep(500);

        motorFlick.setPower(1);  //FIRE 2!!!
        Thread.sleep(800);

        motorFlick.setPower(MOTOR_STOP);// turns off Flick

//        encoderDrive(TURN_SPEED,  5,  0, 3.0); //turns towards beacon

        // Once Start is pressed the robot begins moving forward, and then enters while loop looking for a white line threshold.
        motor_left.setPower(.4);
        motor_right.setPower(.4);

        // run until the white line is seen OR the driver presses STOP;
        // will continue to display Light Level values
        while ( ODSRR.getLightDetected() < WHITE_THRESHOLD) {

            // Continue displaying the light level while we are looking for the line threshold value
            telemetry.addData("RIGHT REAR:",  ODSRR.getLightDetected());
            telemetry.addData("LEFT REAR:",  ODSLR.getLightDetected());
            telemetry.addData("RIGHT FRONT",  ODSRF.getLightDetected());
            telemetry.addData("LEFT FRONT",  ODSLF.getLightDetected());
            telemetry.update();
        }

        // Stop all motors
        motor_left.setPower(MOTOR_STOP);
        motor_right.setPower(MOTOR_STOP);
        sleep(500);     // pause for 1/2 sec to allow motors to fully stop



    }
//////////////////////////
//ENCODER DRIVE METHOD........
//////////////////////////
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motor_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motor_left.setTargetPosition(newLeftTarget);
            motor_right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.\
            motor_left.setPower(Math.abs(speed));
            motor_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (motor_left.isBusy() && motor_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motor_left.getCurrentPosition(),
                        motor_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor_left.setPower(0);
            motor_right.setPower(0);

            // Turn off RUN_TO_POSITION
            motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
