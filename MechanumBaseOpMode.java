package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

/**
 * Created by sgodi on 11/12/2017.
 */

@TeleOp(name = "Mechanum Op Mode", group = "Mechanum")
//@Disabled
public class MechanumBaseOpMode extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    //ModernRoboticsI2cGyro mrGyro;
    //\IntegratingGyroscope gyro;
    double speedFactor = 4;
    private CRServo collector = null;
    private DcMotor winch = null;
    private DcMotor bucket = null;
    private CRServo liftyLift = null;
    public CRServo extendCollector = null;
    public DcMotor pivotCollector = null;
    public DigitalChannel winchSwitch = null;

    boolean check = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware varia=bles. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft"); //frontLeft
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight"); //frontRight
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeft"); //rearLeft
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRight"); //rearRight
        collector = hardwareMap.get(CRServo.class, "collector");
        winch = hardwareMap.get(DcMotor.class, "winch");
        bucket = hardwareMap.get(DcMotor.class, "bucket");
        liftyLift = hardwareMap.get(CRServo.class, "Servo4");
        extendCollector = hardwareMap.get(CRServo.class, "Servo5");
        pivotCollector = hardwareMap.get(DcMotor.class, "pivotCollector");
        winchSwitch = hardwareMap.get(DigitalChannel.class, "winchButton");
        winchSwitch.setMode(DigitalChannel.Mode.INPUT);


        //mrGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        //gyro = (IntegratingGyroscope) mrGyro;


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        winch.setDirection(DcMotorSimple.Direction.FORWARD);
        bucket.setDirection(DcMotorSimple.Direction.FORWARD);


        //Sets encoder modes for motors
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  mrGyro.calibrate();


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //Variables for inputs
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;
        double rightStickY = gamepad1.right_stick_y;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;


        // Diagnostics
        telemetry.addData("Left Stick X input: ", leftStickX);
        telemetry.addData("Left Stick Y input: ", leftStickY);
        telemetry.addData("Right Stick X input: ", rightStickX);
        telemetry.addData("Right Stick X input: ", rightStickY);
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
//        telemetry.clearAll();
//        while (mrGyro.isCalibrating()) {
//            telemetry.addData("Wait", "!");
//            telemetry.update();
//        }
//        telemetry.clearAll();
//        telemetry.addData("Status", "Done");
//        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Variables for inputs
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;
        double rightStickY = gamepad1.right_stick_y;
        double bucketStartPosition = bucket.getCurrentPosition();

        // check to see if we need to move the servo.

        //Calculations for motor power
//        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
//        double r = Math.hypot(leftStickX, leftStickY) / Math.max(Math.abs(Math.sin(robotAngle)), Math.abs(Math.cos(robotAngle)));
//        double rotate = -rightStickX;
//        final double frontLeft = r * Math.cos(robotAngle) + rotate;
//        final double frontRight = r * Math.sin(robotAngle) - rotate;
//        final double rearLeft = r * Math.sin(robotAngle) + rotate;
//        final double rearRight = r * Math.cos(robotAngle) - rotate;
//        if(leftStickX != 0 || leftStickY != 0 || rightStickX != 0) {
//            if (leftStickX > 0.15 || leftStickX < -0.15) {
//                //Motor Power Sets
//                frontLeftDrive.setPower(frontLeft);
//                frontRightDrive.setPower(frontRight);
//                rearLeftDrive.setPower(rearLeft);
//                rearRightDrive.setPower(rearRight);
//            } else =5
//                //Motor Power Sets
//                frontLeftDrive.setPower((frontLeft / speedFactor) - .2);
//                frontRightDrive.setPower(frontRight / speedFactor);
//                rearLeftDrive.setPower(rearLeft / speedFactor);
//                rearRightDrive.setPower(rearRight / speedFactor);
//            }
//        }
//        else
//        {
//            frontLeftDrive.setPower(0);
//            frontRightDrive.setPower(0.005);
//            rearLeftDrive.setPower(0.005);
//            rearRightDrive.setPower(0.005);
//
//        }
        //The Drive chain motors are in fact reversed
        if(gamepad1.left_stick_y > .03 || gamepad1.left_stick_y <-.03)
        {
            if(gamepad1.left_stick_y >.03)
            {
                frontLeftDrive.setPower(-.3);
                frontRightDrive.setPower(-.3);
                rearLeftDrive.setPower(-.3);
                rearRightDrive.setPower(-.3);
                telemetry.addData("Encoder Location",frontLeftDrive.getCurrentPosition());
                telemetry.update();

            }
            else if(gamepad1.left_stick_y < -.03)
            {
                frontLeftDrive.setPower(.3);
                frontRightDrive.setPower(.3);
                rearLeftDrive.setPower(.3);
                rearRightDrive.setPower(.3);
                telemetry.addData("Encoder Location",frontLeftDrive.getCurrentPosition());
                telemetry.update();

            }
            telemetry.clearAll();

        }
        else if(gamepad1.left_stick_x > .03 || gamepad1.left_stick_x <-.03)
        {
            if(gamepad1.left_stick_x >.03)
            {
                frontLeftDrive.setPower(.3);
                frontRightDrive.setPower(-.5);
                rearLeftDrive.setPower(.3);
                rearRightDrive.setPower(-.5);

            }
            else if(gamepad1.left_stick_x < -.03)
            {
                frontLeftDrive.setPower(-.5);
                frontRightDrive.setPower(.3);
                rearLeftDrive.setPower(-.5);
                rearRightDrive.setPower(.3);


            }
        }
        else if (gamepad1.right_stick_x >.03 || gamepad1.right_stick_x <-.03)
        {
            if(gamepad1.right_stick_x >.03)
            {
                frontLeftDrive.setPower(-.7);
                frontRightDrive.setPower(.5);
                rearLeftDrive.setPower(-.7);
                rearRightDrive.setPower(.5);

            }
            else if(gamepad1.right_stick_x < -.03)
            {
                frontLeftDrive.setPower(.5);
                frontRightDrive.setPower(-.7);
                rearLeftDrive.setPower(.5);
                rearRightDrive.setPower(-.7);


            }
        }
        else{
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                rearLeftDrive.setPower(0);
                rearRightDrive.setPower(0);


        }


//        if (leftStickX > 0.03 || leftStickX < -0.1) {
//            //Motor Power Sets
//            frontLeftDrive.setPower(frontLeft);
//            frontRightDrive.setPower(frontRight);
//            rearLeftDrive.setPower(rearLeft);
//            rearRightDrive.setPower(rearRight);
//        }
//            //Motor Power Sets
//
////            if (frontRightDrive.getCurrentPosition() < frontRightDrive.getCurrentPosition()) {
////                if(frontRightDrive.getCurrentPosition() < frontRightDrive.getCurrentPosition() + 20) {
////                    frontLeftDrive.setPower(frontLeft / 4);
//            frontRightDrive.setPower(frontRight / 4);
//            rearLeftDrive.setPower(rearLeft / 4);
//            rearRightDrive.setPower(rearRight / 4);
//            telemetry.clearAll();
//            telemetry.addData("Status: ", "Stage 1");
//            telemetry.update();
//
////                }
////                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                if(frontRightDrive.getCurrentPosition() < frontRightDrive.getCurrentPosition() + 10){
////                    frontLeftDrive.setPower(frontLeft / 2);
////                    frontRightDrive.setPower(frontRight / 2);
////                    rearLeftDrive.setPower(rearLeft / 2);
////                    rearRightDrive.setPower(rearRight / 2);
////                    telemetry.clearAll();
////                    telemetry.addData("Status: ", "Stage 2");
////                    telemetry.update();
////
////
////                }
////            }
////            frontLeftDrive.setPower(frontLeft);
////            frontRightDrive.setPower(frontRight);
////            rearLeftDrive.setPower(rearLeft);
////            rearRightDrive.setPower(rearRight);
////            telemetry.clearAll();
////            telemetry.addData("Status: Complete", "Stage 3");
////            telemetry.update();
////        }
            if (gamepad2.right_bumper) {
                collector.setPower(.85);
            } else if (gamepad2.left_bumper) {
                collector.setPower(-.85);
            } else {
                collector.setPower(0);
            }
           // boolean isPressed = winchSwitch.getState();
            if (gamepad2.dpad_up) {
                if(winchSwitch.getState()){
                    winch.setPower(0);
                }
                else
                {
                    winch.setPower(1);
                }

            } else if (gamepad2.dpad_down ) {
                winch.setPower(-1);
            }
            else{
                winch.setPower(0);
            }
            if (gamepad1.right_bumper) {
                bucket.setPower(-.2);
            } else if (gamepad1.left_bumper) {
                telemetry.clearAll();
                if (bucket.getCurrentPosition() > bucketStartPosition) {
                    bucket.setPower(.2);
                } else {
                    bucket.setPower(0);
                    telemetry.addData("Status: ", "Bucket below threshold");
                    telemetry.update();
                }

            } else {
                bucket.setPower(0);
            }
            if (gamepad2.x) {
                liftyLift.setPower(.3);
            } else if (gamepad2.b) {
                liftyLift.setPower(-.3);
            } else {
                liftyLift.setPower(.01);
            }
            boolean isUp = false;
            if (gamepad1.dpad_up) {
                extendCollector.setPower(1);
                isUp = true;
            } else if (gamepad1.dpad_down) {
                extendCollector.setPower(-1);
                isUp = false;
            } else {
                if(isUp)
                {
                    extendCollector.setPower(.3);
                }
                else
                {
                    extendCollector.setPower(0);
                }

            }
            if (gamepad1.b) {
                pivotCollector.setPower(-.3);
            } else if (gamepad1.x) {
                pivotCollector.setPower(1);
            } else {
                pivotCollector.setPower(.03);
            }

//        try {
//            // Diagnostics
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Gyro X Axis ", mrGyro.rawX());
//            telemetry.addData("Gyro Y Axis ", mrGyro.rawY());
//            telemetry.addData("Gyro Z Axis ", mrGyro.rawZ());
//            telemetry.addData("Integrated Z Axis ", mrGyro.getIntegratedZValue());
//            telemetry.addData("Heading ", mrGyro.getHeading());
//            telemetry.update();
//        } catch (Exception e) {
//            telemetry.clearAll();
//            telemetry.addData("Exception thrown in loop method", e.getMessage());
//            telemetry.update();
//        }
        }

    }
