/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="5520 TeleOp", group="Linear Opmode")
@Disabled
public class Telepole extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor frontLeftDrive = null;
    double leftStickX = gamepad1.left_stick_x;
    double leftStickY = gamepad1.left_stick_y;
    double rightStickX = -gamepad1.right_stick_x; //added negitive
    double rightStickY = gamepad1.right_stick_y;
    double r = Math.hypot(leftStickX, -leftStickY);
    double robotAngle = Math.atan2(-leftStickY, leftStickX) - Math.PI / 4;
    double rotate = -rightStickX;
    final double frontLeft = r * Math.cos(robotAngle) + rotate;
    final double frontRight = r * Math.sin(robotAngle) - rotate;
    final double rearLeft = r * Math.sin(robotAngle) + rotate;
    final double rearRight = r * Math.cos(robotAngle) - rotate;
    double leftFrontMotorPower = 0;
    double speedFactor = 1;
    boolean x2 = gamepad2.x;
    boolean y2 = gamepad2.y;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rearLeftDrive  = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(x2)
            {
                speedFactor = ((double)(3/2));
            }
            else if(y2)
            {
                speedFactor = 2; //made 30% slower (changed from 1.5 to 1.05)
            }

            if(leftStickX != 0 || leftStickY != 0 || rightStickX != 0) {
                if (leftStickX > 0.1 || leftStickX < -0.1) {
                    //Motor Power Sets
                    frontLeftDrive.setPower(frontLeft);
                    frontRightDrive.setPower(frontRight);
                    rearLeftDrive.setPower(rearLeft);
                    rearRightDrive.setPower(rearRight);
                } else {
                    //Motor Power Sets
                    frontLeftDrive.setPower(frontLeft / speedFactor);
                    leftFrontMotorPower = frontLeft / speedFactor;
                    frontRightDrive.setPower(frontRight / speedFactor);
                    rearLeftDrive.setPower(rearLeft / speedFactor);
                    rearRightDrive.setPower(rearRight / speedFactor);
                }
            }
            else
            {
                frontLeftDrive.setPower(0.005);
                frontRightDrive.setPower(0.005);
                rearLeftDrive.setPower(0.005);
                rearRightDrive.setPower(0.005);

            }






        }
    }
}
