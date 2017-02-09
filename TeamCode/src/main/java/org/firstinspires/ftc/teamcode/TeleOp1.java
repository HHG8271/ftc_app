package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name=" TeleOp1", group="Examples")  // @Autonomous(...) is the other common choice

public class TeleOp1 extends LinearOpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //motors
    DcMotor SpinnyL = null;
    DcMotor SpinnyR = null;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorFlick = null;
    DcMotor motorConveyor = null;
    DcMotor motorPusher = null;
    Servo hook = null;
    public ColorSensor color_sensor;
    public OpticalDistanceSensor ODSRR;
    public OpticalDistanceSensor ODSRF;
    public OpticalDistanceSensor ODSLF;
    public OpticalDistanceSensor ODSLR;


    //servos
    // Servo servoHandL = null;
    //Servo servoHandR = null;

    //Create and set default hand positions variables. To be determined based on your build
    double CLOSED = 0.8;
    double OPEN = 0.2;
    boolean Reverse = false;
    boolean Standard = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        color_sensor = hardwareMap.colorSensor.get("color_sensor");
        color_sensor.enableLed(false);
        SpinnyL = hardwareMap.dcMotor.get("spinnyl");
        SpinnyR = hardwareMap.dcMotor.get("spinnyr");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorFlick = hardwareMap.dcMotor.get("motorFlick");
        motorConveyor = hardwareMap.dcMotor.get("motorConveyor");
        motorPusher = hardwareMap.dcMotor.get("motorPusher");
        hook = hardwareMap.servo.get("hook");
        ODSLR = hardwareMap.opticalDistanceSensor.get("LR");
        ODSLF = hardwareMap.opticalDistanceSensor.get("LF"); /// SWITCH CONFIGURATION FROM LEFT() TO RIGHT()
        ODSRF = hardwareMap.opticalDistanceSensor.get("RF");
        ODSRF = hardwareMap.opticalDistanceSensor.get("RR");

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
        hook.setPosition(.65);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

           /* ***********************
             * TeleOp Code Below://
             *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("red",color_sensor.red());
            telemetry.addData("blue",color_sensor.blue());
            telemetry.addData("RIGHT REAR:",  ODSRR.getLightDetected());
            telemetry.addData("LEFT REAR:",  ODSLR.getLightDetected());
            telemetry.addData("RIGHT FRONT",  ODSRF.getLightDetected());
            telemetry.addData("LEFT FRONT",  ODSLF.getLightDetected());

            telemetry.update();

            // tank drive set to gamepad1 joysticks
            //(note: The joystick goes negative when pushed forwards)
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);


            if (gamepad1.right_bumper) {
                motorPusher.setPower(0.5);
            }

             else if (gamepad1.left_bumper) {
                motorPusher.setPower(-0.5);

            } else {
                motorPusher.setPower(0);
            }
            if (gamepad1.b) {
                hook.setPosition(.1);
            }

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
            if (gamepad2.b) {
                motorConveyor.setPower(1);
            }
            if (gamepad2.left_bumper) {
                SpinnyL.setPower(-.4);


            } else if (gamepad2.right_bumper) {
                SpinnyR.setPower(-.4);
            } else {

                SpinnyL.setPower(gamepad2.right_trigger);
                SpinnyR.setPower(gamepad2.left_trigger);
            }


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}






