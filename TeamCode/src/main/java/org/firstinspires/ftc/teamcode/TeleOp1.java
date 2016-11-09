package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name=" TeleOp1", group="Examples")  // @Autonomous(...) is the other common choice

public class TeleOp1 extends LinearOpMode
{
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorFlick = null;
    DcMotor motorConveyor = null;

    //servos
    // Servo servoHandL = null;
    //Servo servoHandR = null;

    //Create and set default hand positions variables. To be determined based on your build
    double CLOSED = 0.2;
    double OPEN = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        motorLeft  = hardwareMap.dcMotor.get("motor_left");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorFlick = hardwareMap.dcMotor.get("motorFlick");
        motorConveyor = hardwareMap.dcMotor.get("motorConveyor");
        // servoHandL = hardwareMap.servo.get("servoHandL"); //assuming a pushBot configuration of two servo grippers
        //servoHandR = hardwareMap.servo.get("servoHandR");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorFlick.setDirection(DcMotor.Direction.FORWARD);
        motorConveyor.setDirection(DcMotor.Direction.FORWARD);// Can change based on motor configuration

        //Set servo hand grippers to open position.
        // servoHandL.setPosition(OPEN);
        //servoHandR.setPosition(OPEN);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

           /* ***********************
             * TeleOp Code Below://
             *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // tank drive set to gamepad1 joysticks
            //(note: The joystick goes negative when pushed forwards)
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            // Arm Control - Uses dual buttons to control motor direction
            if (gamepad2.a) {
                motorFlick.setPower(1); // if both Bumper + Trigger, then negative power, runs arm down
            } else {
                motorFlick.setPower(0);  // else trigger positive value, runs arm up
            }
            if (gamepad2.y) {
                motorConveyor.setPower(-1);
            }
            if (gamepad2.x) {
                motorConveyor.setPower(0);
            }
            if (gamepad2.b)
            {
                motorConveyor.setPower(1);
            }


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}


