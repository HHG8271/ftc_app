/*
package org.firstinspires.ftc.teamcode.ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BeastHardwareSetup;
import org.firstinspires.ftc.teamcode.ExampleCode.MyBotHardwareSetup;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

*/
/**
 * Basic example for using Vision program in autonomous code
 * Creating TEST BED example program
 * with demo programing
 *
 *//*



@TeleOp(name = "robBeaconAuto", group = "Test")

public class RobBeaconAuto extends LinearVisionOpMode() {

    BeastHardwareSetup savage = new BeastHardwareSetup(); //set up remote to robot hardware configuration

    // @Override
    public void runOpMode() throws InterruptedException {
        // VARIABLES for reading Beacon
        String beaconVal;
        String beaconValNext;
        String[] beaconVals;  //string array variable

        //Create and set desired variables, i.e. default hand positions. To be determined based on your build
        final  double CLOSED = 0.2;
        final  double OPEN = 0.8;
        final  double NEUTRAL = 0.5;
        final  double MOTOR_STOP = 0.0; // sets motor power to zero



        //waitForVisionStart();

        // CONFIGURE PHONE CAMERA SETTINGS
        this.setCamera(Cameras.SECONDARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);

        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(true);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        beaconVals = readBeacon(); // run 'readBeacon' method (below) and store as 'beaconVals'

        telemetry.addData("Beacon Reading", beaconVals);  // displays final reading . what it will act on.
        telemetry.update();

        waitForStart();

        //
        // Place first set of autonomous code here. Get into position for reading the beacon
        DriveForwardTime(DRIVE_POWER, 4000);
        TurnLeft(DRIVE_POWER, 1000);
        StopDrivingTime(2000);
        //

        beaconVals = readBeacon(); // run 'readBeacon' method (below) and store as 'beaconVals'

        //
        // Get into position for pressing beacon
        DriveForwardTime(DRIVE_POWER, 2000);
        TurnLeft(DRIVE_POWER, 1000);
        StopDrivingTime(2000);
        //



        // run your beacon button push based on reading taken
        if (beaconVals[0].equals("red")) { //reads first array position, noted as zero, as "red" Array is... "???,???"

            //push red button

        }
        else {

            //push blue button

        }

        Thread.sleep(3000); // keeps motorArm running for 3 sec then stops


        //
        // Place any additional set of autonomous code here.
        DriveForwardTime(DRIVE_POWER, 2000);
        TurnLeft(DRIVE_POWER, 1000);
        StopDrivingTime(2000);
        //

    }//runOpMode


    // Below is Method used to read the beacon value

    public String[] readBeacon () throws InterruptedException{

        String beaconValNext = "???,???";
        String beaconVal = "???,???";
        String[] beaconVals = beaconVal.split(",");
        while (!(beaconVals[0].equals("red") || beaconVals[0].equals("blue")) || !beaconValNext.equals(beaconVal)) {  // waits until accurate reading Not ???
            Thread.sleep(300);
            beaconVal = beaconValNext;
            beaconValNext = beacon.getAnalysis().getColorString();
            beaconVals = beaconVal.split(",");
            telemetry.addData("Left", beaconVals[0]);
            telemetry.update();
        }
        telemetry.addData("Left Beacon Reading", beaconVal);  // displays final reading . what it will act on.
        telemetry.update();
        return beaconVals;

    }//readBeacon

    */
/** Below: Basic Drive Methods used in Autonomous code...**//*

    //set Drive Power variable
    double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {

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

    }

    public void TurnRight(double power, long time) throws InterruptedException
    {
        TurnLeft(-power, time);
    }

    public void RaiseArm()
    {
       // robot.servoHandL.setPosition(.8); //note: uses servo instead of motor.
    }

    public void LowerArm()
    {
      //  robot.servoHandR.setPosition(.2);
    }

}//MyBasicVisionSample
*/
