

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Basic", group = "Autonomous")
//@Disabled
public class Autonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private CRServo collector = null;
    private DcMotor winch = null;
    private DcMotor bucket = null;
    private CRServo liftyLift = null;
    public CRServo extendCollector = null;
    DigitalChannel winchButton;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro mrGyro;
    private Servo name = null;
    private GoldAlignDetector detector;
    Orientation angles;
    public DcMotor pivotCollector = null;
    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft"); //frontLeft
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight"); //frontRight
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeft"); //rearLeft
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRight"); //rearRight
        mrGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        collector = hardwareMap.get(CRServo.class, "collector");
        winch = hardwareMap.get(DcMotor.class, "winch");
        bucket = hardwareMap.get(DcMotor.class, "bucket");
        liftyLift = hardwareMap.get(CRServo.class, "Servo4");
        extendCollector = hardwareMap.get(CRServo.class, "Servo5");
        gyro = (IntegratingGyroscope) mrGyro;
        winchButton = hardwareMap.get(DigitalChannel.class, "winchButton");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        winchButton.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Gyro: ", "Do not touch Gyro Calibrating");
        telemetry.update();
        mrGyro.calibrate();
        while (mrGyro.isCalibrating()) {
        }
        telemetry.clearAll();

//        boolean isPressed = winchButton.getState();
//        while(isPressed == false)
//        {
//            winch.setPower(0.3);
//        }
//        strafeDistance(1, -2, Math.toRadians(45), 3000);
//        strafeDistance(1, 5, Math.toRadians(90), 5000);
//        boolean isFound=false;
//        for(double d=0; d<10; d=d +.1){
//            if(detector.getAligned() == false && !isFound){
//                strafeDistance(1, 2, Math.toRadians(135), 2000);
//                strafeDistance(1, -2, Math.toRadians(135), 2000);
//                isFound = true;
//            }
//            strafeDistance(1, -0.1,Math.toRadians(45), 2000);
//        }
//        myTurnAbsolute(45,.3);
//        strafeDistance(1, 20, Math.toRadians(45), 10000);
//        extendCollector.setPower(1);
//        sleep(1000);
//        extendCollector.setPower(0);
//        liftyLift.setPower(1);
//        sleep(3000);
//        liftyLift.setPower(-0.5);
//        telemetry.addData("Status: ", "Reached straight");
//        telemetry.update();
//
//        myDriveStraight(1400);
//        telemetry.clearAll();
//                 ------------------------------------------------------------------------------------------------------------------
//            telemetry.addData("Status: ", "Reached turn");
//            telemetry.update();
//            sleep(1000);
//
//            myTurnAbsolute(80, .15); //go about 11deg less than target andf also use .15 turn speed positive target is left
//            telemetry.clearAll();

//            myDriveStraight(50, .3); //positive is straight

//            myTurnAbsolute(0, .15); //go about 11deg less than target andf also use .15 turn speed positive target is left
//            myDriveStraight(70,.3);
//          ----------------------------------------------------------------------------
//        dropAndStop();
//        myTurnAbsolute(25,.15);
//        myDriveStraight(300,.3);
//        myTurnAbsolute(0,.15);
//        locateAndKnockBall(50);
//        myTurnAbsolute(0,.15);
//        myDriveStraight(600,.5);
//        //at this point you drop the mineral lol
        dropAndStop();
        strafeTime(500,0.2);
        myTurnAbsolute(180,.15);
        myDriveStraight(30,.3);
        myTurnAbsolute(0,.15);
        locateAndKnockBall(30);


        // run until the end of the match (driver presses STOP)


    }
    public void locateAndKnockBall(int knockDistance) throws InterruptedException
    {
        telemetry.clearAll();
        telemetry.addData("Status:", "Looking for Cube");
        telemetry.update();
        while (detector.getXPosition() <= 300) //maybe fix to get alligned depending on the field testing
        {
            rearLeftDrive.setPower(.15);
            rearRightDrive.setPower(.15);
            frontRightDrive.setPower(.15);
            frontLeftDrive.setPower(.15);
            sleep(500);
            waitOneFullHardwareCycle();
        }
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        myTurnAbsolute(90,.15);
        myDriveStraight(knockDistance,.15);

    }

    private void myTurnAbsolute(int target, double userTurnSpeed) {
        double currentPosition = mrGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double turnSpeed = userTurnSpeed;
        if (target < currentPosition) {
            while (target <= currentPosition) {
                frontLeftDrive.setPower(-turnSpeed);
                rearLeftDrive.setPower(-turnSpeed);
                frontRightDrive.setPower(turnSpeed);
                rearRightDrive.setPower(turnSpeed);
                currentPosition = mrGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


                telemetry.clearAll();
                telemetry.addData("Left Gyro Status", currentPosition);
                telemetry.update();

            }
        } else if (target > currentPosition) {
            while (target >= currentPosition) {
                frontLeftDrive.setPower(turnSpeed);
                rearLeftDrive.setPower(turnSpeed);
                frontRightDrive.setPower(-turnSpeed);
                rearRightDrive.setPower(-turnSpeed);
                currentPosition = mrGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


                telemetry.clearAll();
                telemetry.addData("Right Gyro Status", currentPosition);
                telemetry.update();
            }

        } else {
            telemetry.clearAll();
            telemetry.addData("Gyro Status", "Error");
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
        telemetry.clearAll();
    }

    public void dropAndStop()
    {
        while(winchButton.getState() == false)
        {
            winch.setPower(.3); //reverse this if it goes the wrong way
        }
    }


    public void stopRobot(int timeoutValue)
    {
        frontLeftDrive.setPower(-.2);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(-.2);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(-.2);
        rearRightDrive.setPower(0);
        rearLeftDrive.setPower(-.2);
        rearLeftDrive.setPower(0);

        sleep(timeoutValue);

        telemetry.clearAll();
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();

    }
    public void myDriveStraight(int target, double speed)
    {
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setTargetPosition(target);
        rearRightDrive.setTargetPosition(target);
        frontRightDrive.setTargetPosition(target);
        frontLeftDrive.setTargetPosition(target);
        runtime.reset();
        rearLeftDrive.setPower(Math.abs(speed));
        rearRightDrive.setPower(Math.abs(speed));
        frontRightDrive.setPower(Math.abs(speed));
        frontLeftDrive.setPower(Math.abs(speed));
        while(rearLeftDrive.isBusy() && opModeIsActive()) {
            //Loop body can be empty
        }
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }


    public void initialize() {


        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!
    }
    public void myEncoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;


/*    public void strafeDistance(double speed, double inches, double theta, double timeoutS) {
        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;
        double xInches = inches * Math.cos(theta);
        double yInches = inches * Math.sin(theta);
        double xSpeed = speed * Math.cos(theta);
        double ySpeed = speed * Math.sin(theta);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (xInches * COUNTS_PER_INCH);
            newRearLeftTarget = rearLeftDrive.getCurrentPosition() + (int) (yInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (yInches * COUNTS_PER_INCH);
            newRearRightTarget = rearRightDrive.getCurrentPosition() + (int) (xInches * COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            rearLeftDrive.setTargetPosition(newRearLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            rearRightDrive.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(xSpeed));
            rearLeftDrive.setPower(Math.abs(ySpeed));
            frontRightDrive.setPower(Math.abs(ySpeed));
            rearRightDrive.setPower(Math.abs(xSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget,
                        newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        frontLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), rearLeftDrive.getCurrentPosition(),
                        rearRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            rearLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/

    }

    public void strafeTime(int timeoutValue, double speed){
        frontLeftDrive.setPower(-speed);
        frontRightDrive.setPower(speed);
        rearRightDrive.setPower(speed);
        rearLeftDrive.setPower(-speed);

        sleep(timeoutValue);

        telemetry.clearAll();
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }

}
