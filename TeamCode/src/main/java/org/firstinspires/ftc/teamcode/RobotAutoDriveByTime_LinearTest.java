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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
//@Disabled
public class RobotAutoDriveByTime_LinearTest extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor MotorFR = null;
    private DcMotor MotorBR = null;
    private DcMotor MotorFL = null;
    private DcMotor MotorBL = null;
    private DcMotor Shoulder = null;
    private DcMotor Elbow = null;
    private Servo Wrist = null;
    private Servo Gripper = null;

    double currentServoPosition;


    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double STRAFE_SPEED = 0.5;

    double xTarget = 9.65;
    double zTarget = 11.69;


    @Override
    public void runOpMode() {


        // Initialize the drive system variables.
        MotorFR = hardwareMap.get(DcMotor.class, "MotorFR");
        MotorBR = hardwareMap.get(DcMotor.class, "MotorBR");
        MotorFL = hardwareMap.get(DcMotor.class, "MotorFL");
        MotorBL = hardwareMap.get(DcMotor.class, "MotorBL");
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        Elbow = hardwareMap.get(DcMotor.class, "Elbow");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Gripper = hardwareMap.get(Servo.class, "Gripper");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        MotorFL.setDirection(DcMotor.Direction.REVERSE);
        MotorBL.setDirection(DcMotor.Direction.REVERSE);
        MotorFR.setDirection(DcMotor.Direction.FORWARD);
        MotorBR.setDirection(DcMotor.Direction.FORWARD);
        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentServoPosition = .5;
        Wrist.setPosition(currentServoPosition);
        Gripper.setPosition(1);
        calculationIK(xTarget,zTarget);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.
        // Step 1:  Strafe for 1.5 seconds
        MotorFR.setPower(STRAFE_SPEED);
        MotorBR.setPower(-STRAFE_SPEED);
        MotorFL.setPower(-STRAFE_SPEED);
        MotorBL.setPower(STRAFE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 1:  Rotate for 1.5 seconds
        MotorFR.setPower(TURN_SPEED);
        MotorBR.setPower(TURN_SPEED);
        MotorFL.setPower(-TURN_SPEED);
        MotorBL.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        xTarget = 10.84;
        zTarget = 57.16;
        calculationIK(xTarget,zTarget);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        Gripper.setPosition(0.8); // Open
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        // Step 2:  Stop
        MotorFR.setPower(0);
        MotorBR.setPower(0);
        MotorFL.setPower(0);
        MotorBL.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void calculationIK(double xTarget, double zTarget) {
        double L1 = 28.58;
        double L2 = 33.02;
        // Normal inverse kinematics calculation

        double distanceToTarget = Math.sqrt(xTarget * xTarget + zTarget * zTarget);

        if (distanceToTarget > (L1 + L2)) {
            telemetry.addData("Error", "Target is out of reach.");
            telemetry.update();
        } else {
            double cosTheta2 = (L1 * L1 + L2 * L2 - distanceToTarget * distanceToTarget) / (2 * L1 * L2);
            double theta2 = Math.acos(cosTheta2);

            double k1 = L1 + L2 * Math.cos(theta2);
            double k2 = L2 * Math.sin(theta2);
            double theta1 = Math.atan2(zTarget, xTarget) - Math.atan2(k2, k1);

            double theta1Deg = Math.toDegrees(theta1);
            double theta2Deg = Math.toDegrees(theta2) - 180;

            int ShoulderTargetPos = (int) (theta1Deg * 58.678);
            int ElbowTargetPos = (int) (theta2Deg * 30.9576);

            Shoulder.setTargetPosition(ShoulderTargetPos);
            Elbow.setTargetPosition(ElbowTargetPos);
            Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shoulder.setPower(0.8);
            Elbow.setPower(0.8);

            telemetry.addData("Theta1", theta1Deg);
            telemetry.addData("Theta2", theta2Deg);
            telemetry.addData("Shoulder Target", ShoulderTargetPos);
            telemetry.addData("Elbow Target", ElbowTargetPos);
            telemetry.update();
        }

    }
}