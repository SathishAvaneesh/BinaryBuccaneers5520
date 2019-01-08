package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;


/**
 * Created by sgodi on 11/17/2017.
 */
@Autonomous(name = "New Blue Turn Autonomous", group = "Encoder")
@Disabled
public class NewBlueTurnAutonomous extends LinearOpMode
{
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    // Declare OpMode members.
    private ElapsedTime runtime         = new ElapsedTime();
    private DcMotor frontLeftDrive      = null;
    private DcMotor frontRightDrive     = null;
    private DcMotor rearLeftDrive       = null;
    private DcMotor rearRightDrive      = null;
    private Servo armServo              = null;

    private ColorSensor colorSensor     = null;
    private Servo bottomRightServo      = null;
    private Servo bottomLeftServo       = null;
    private Servo topRightServo         = null;
    private Servo topLeftServo          = null;
    private DcMotor spool               = null;

    private static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED             = 0.1;
    private static final double TURN_SPEED              = 0.1;
    String vuforiaOutput = "";

    //Variables for Integrated Gyro Sensor
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    // Other Variables
    private VuforiaLocalizer vuforia;
    private final int debugTime         = 1000;
    private final int spoolUpTime       = 1250;
    private final int spoolDownTime     = 1000;
    private final int spoolReleaseTime  = 500;
    private double extraDistance        = 3.0; //compensation for driving backward
    private double vuMarkScanDistance   = 0.0; //saves distance depending on vumark. (inches)


    @Override
    public void runOpMode() throws InterruptedException
    {

            /*`
             * Initialize the drive system variables.
             */
            frontLeftDrive      = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRightDrive     = hardwareMap.get(DcMotor.class, "frontRight");
            rearLeftDrive       = hardwareMap.get(DcMotor.class, "rearLeft");
            rearRightDrive      = hardwareMap.get(DcMotor.class, "rearRight");
            armServo            = hardwareMap.get(Servo.class, "armServo");
            bottomRightServo    = hardwareMap.get(Servo.class, "bottomRight");
            bottomLeftServo     = hardwareMap.get(Servo.class, "bottomLeft");
            topRightServo       = hardwareMap.get(Servo.class, "topRight");
            topLeftServo        = hardwareMap.get(Servo.class, "topLeft");
            spool               = hardwareMap.get(DcMotor.class, "spool");
            colorSensor         = hardwareMap.get(ColorSensor.class, "color");


            /*
             * Most robots need the motor on one side to be reversed to drive forward
             * Reverse the motor that runs backwards when connected directly to the battery
             */
            frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

            BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
            parametersGyro.loggingEnabled = true;
            parametersGyro.loggingTag     = "IMU";
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parametersGyro);

            BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

            // Save the calibration data to a file. You can choose whatever file
            // name you wish here, but you'll want to indicate the same file name
            // when you initialize the IMU in an opmode in which it is used. If you
            // have more than one IMU on your robot, you'll of course want to use
            // different configuration file names for each.
            String filename = "BNO055IMUCalibration.json";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, calibrationData.serialize());
            telemetry.log().add("saved to '%s'", filename);

            parametersGyro.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parametersGyro.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parametersGyro.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parametersGyro.loggingEnabled      = true;
            parametersGyro.loggingTag          = "IMU";
            parametersGyro.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parametersGyro);
            composeTelemetry();

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();
            telemetry.addData("Now do this: ", "WAIT UNTIL TELEMETRY CHANGES TO CLICK START");
            telemetry.update();

            // Reset Encoders
            telemetry.addData("Status: ", "Resetting Encoders");
            resetEncoders();
            telemetry.update();
            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(), rearLeftDrive.getCurrentPosition(), rearRightDrive.getCurrentPosition());
            telemetry.update();

            // Initializing Vuforia
            telemetry.addData("Status: ", "Initializing Vuforia");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parametersVuforia = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parametersVuforia.vuforiaLicenseKey = "AVA45pf/////AAAAGW2rsD/HnEFGnNiq4IC6kOJLQr1uzR6x6AhdGEYQXgIkm3J0HR6UEyIO5VpdOBjfIXZyG7/6f97dRq745TzVaXJYOikIVmRyqFxksoC4G6DOtc6ZFSAZMU45YmIA5X4FwgQ/bWyUeAWX215Qmj2qI5pmyuRzA7+fyD+KQRW6hMaJNR0feXlqltgYENB/Hc5kQVSJDDPuaN+Mvji31tF2XEur/zzvqhAvqXoSW73J5zXIRynt/T5x13JtxsgHuPno+j/75SGkEo5VLhZKtc1aICrQ1yBmmLkX5gAT81o8yo3bvWcxuhOO1LZE/9aEVKP0tY8bKzHRL1ynrXY7BwGP4wR87DC3kw0CfMfWOcLQjGWf";
            parametersVuforia.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

            this.vuforia = ClassFactory.createVuforiaLocalizer(parametersVuforia);
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            // Identify that Initialization has been Completed
            telemetry.addData("Press Play to Start.", "");
            telemetry.update();

//------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            relicTrackables.activate();
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


            // Grab and Lift the Starting Glyph
            // Process: lift, open, lower, grab, lift
            controlSpool(true, spoolUpTime);
            openGrabber();
            controlSpool(false, spoolDownTime);
            closeGrabber();
            sleep(debugTime);
            controlSpool(true, spoolUpTime);

            // Drop the Arm Servo and scan the color
            telemetry.addData("Status: ", "Got past wait for start");
            telemetry.update();
            armServo.setPosition(.55); //change on position of relic grabber from .92 to .97 to 0.03 to .95 to .55
            telemetry.addData("Status: ", "Scanning color");
            telemetry.update();

            if (colorSensor.red()  > colorSensor.blue() + 10)
            {
                closeGrabber();

                telemetry.addData("Status: ", "Scanning red");
                sleep(debugTime);

                telemetry.addData("red: ", colorSensor.red());
                telemetry.addData("blue: ", colorSensor.blue());
                telemetry.update();

                sleep(debugTime);

                driveForwardDistance(DRIVE_SPEED,2,2,10);

                extraDistance = 0;

                turnLeft(0.05, 10);
                turnRight(0.05, 10);
            }
            else if ( colorSensor.blue() > colorSensor.red() + 10)
            {
                closeGrabber();

                telemetry.addData("Status: ", "Scanning blue");
                sleep(debugTime);

                telemetry.addData("red: ", colorSensor.red());
                telemetry.addData("blue: ", colorSensor.blue());
                telemetry.update();

                sleep(debugTime);

                //driveForwardDistance(DRIVE_SPEED,-2,-2,10);
                turnRight( 0.05,10);
                turnLeft(0.05, 10);
            }
            else
            {
                closeGrabber();

                telemetry.addData("Status: ", "no color sensed");
                sleep(debugTime);

                telemetry.addData("red: ", colorSensor.red());
                telemetry.addData("blue: ", colorSensor.blue());

                extraDistance = 0.0;

            }

            sleep(debugTime);

            armServo.setPosition(0.0);

            //move a set distance
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("vumark: ", vuMark);
            telemetry.update();
            sleep(debugTime);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT)
                {
                    vuMarkScanDistance = 10;
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER)
                {
                    vuMarkScanDistance = 10;
                }
                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                {
                    vuMarkScanDistance = 10;
                }
            }
             else
            {
                vuMarkScanDistance = 10;
            }

            closeGrabber();

            driveForwardDistance(DRIVE_SPEED, vuMarkScanDistance + extraDistance, vuMarkScanDistance + extraDistance, 10);

            turnRight(90);

            driveForwardDistance(DRIVE_SPEED, 2, 2, 10);

            controlSpool(false, spoolReleaseTime);
            openGrabber();

            driveForwardDistance(DRIVE_SPEED, -2, -2, 10);
            driveForwardDistance(DRIVE_SPEED, 2.5, 2.5, 10);
            driveForwardDistance(DRIVE_SPEED, -2, -2, 10);

            // Turn and collect more glyphs from the pile
            //Not for this coming Qualifier
    }


    public void controlSpool(boolean isUp, int moveTime)
    {
        final double spoolUpPower = -0.5;
        final double spoolDownPower = 0.5;

        if (isUp)
        {
            spool.setPower(spoolUpPower);
            sleep(moveTime);
            spool.setPower(0.005);
        }
        else
        {
            spool.setPower(spoolDownPower);
            sleep(moveTime);
            spool.setPower(0.005);
        }
    }

    public void turnLeft(float endingAngle)
    {

        telemetry.update();
        endingAngle = endingAngle;

        while (!(angles.firstAngle < endingAngle-5 && angles.firstAngle > endingAngle+10))
        {
            telemetry.update();

            frontLeftDrive.setPower(-TURN_SPEED);
            rearLeftDrive.setPower(-TURN_SPEED);
            frontRightDrive.setPower(TURN_SPEED);
            rearRightDrive.setPower(TURN_SPEED);

        }

        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public void turnLeft(double turnSpeed, float endingAngle)
    {

        telemetry.update();
        endingAngle = endingAngle;

        while (!(angles.firstAngle < endingAngle-5 && angles.firstAngle > endingAngle+10))
        {
            telemetry.update();

            frontLeftDrive.setPower(-turnSpeed);
            rearLeftDrive.setPower(-turnSpeed);
            frontRightDrive.setPower(turnSpeed);
            rearRightDrive.setPower(turnSpeed);

        }

        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public void turnRight(float endingAngle)
    {
        telemetry.update();
        endingAngle = -endingAngle;

        while (!(angles.firstAngle < endingAngle+5 && angles.firstAngle > endingAngle-10))
        {
            telemetry.update();

            frontLeftDrive.setPower(TURN_SPEED);
            rearLeftDrive.setPower(TURN_SPEED);
            frontRightDrive.setPower(-TURN_SPEED);
            rearRightDrive.setPower(-TURN_SPEED);

        }

        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public void turnRight(double turnSpeed, float endingAngle)
    {

        telemetry.update();
        endingAngle = -endingAngle;

        while (!(angles.firstAngle < endingAngle+5 && angles.firstAngle > endingAngle-10))
        {
            telemetry.update();

            frontLeftDrive.setPower(turnSpeed);
            rearLeftDrive.setPower(turnSpeed);
            frontRightDrive.setPower(-turnSpeed);
            rearRightDrive.setPower(-turnSpeed);

        }

        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
    }

    public void closeGrabber()
    {
        bottomRightServo.setPosition(0.5);
        bottomLeftServo.setPosition(0.45);
        topRightServo.setPosition(0.65);
        topLeftServo.setPosition(0.55);
    }

    public void openGrabber()
    {
        bottomRightServo.setPosition(0.75);
        bottomLeftServo.setPosition(0.2);
        topRightServo.setPosition(0.85);
        topLeftServo.setPosition(0.3);
    }


    public void resetEncoders()
    {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Drives Certain Distance
    public void driveForwardDistance(double speed, double leftInches, double rightInches, double timeoutS)
    {
        resetEncoders();

        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRearLeftTarget = rearLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRearRightTarget = rearRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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

            float initialAngle = angles.firstAngle;

            frontLeftDrive.setPower(Math.abs(speed));
            rearLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            rearRightDrive.setPower(Math.abs(speed));

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
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget,
                        newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), rearLeftDrive.getCurrentPosition(),
                        rearRightDrive.getCurrentPosition());
                telemetry.update();

                if (angles.firstAngle < initialAngle - 5)
                {
                    frontLeftDrive.setPower(Math.abs(speed) + 0.1);
                    rearLeftDrive.setPower(Math.abs(speed) + 0.1);
                    frontRightDrive.setPower(Math.abs(speed) - 0.1);
                    rearRightDrive.setPower(Math.abs(speed) - 0.1);
                }
                else if (angles.firstAngle > initialAngle + 5)
                {
                    frontLeftDrive.setPower(Math.abs(speed) - 0.1);
                    rearLeftDrive.setPower(Math.abs(speed) - 0.1);
                    frontRightDrive.setPower(Math.abs(speed) + 0.1);
                    rearRightDrive.setPower(Math.abs(speed) + 0.1);
                }
                else
                {
                    frontLeftDrive.setPower(Math.abs(speed));
                    rearLeftDrive.setPower(Math.abs(speed));
                    frontRightDrive.setPower(Math.abs(speed));
                    rearRightDrive.setPower(Math.abs(speed));
                }
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
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
