/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="InverseKinematicsTest", group="Linear OpMode")
//@Disabled
public class InverseKinematicsTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MotorFL = null;
    private DcMotor MotorBL = null;
    private DcMotor MotorFR = null;
    private DcMotor MotorBR = null;
    private DcMotor Shoulder = null;
    private DcMotor Elbow = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        MotorFL = hardwareMap.get(DcMotor.class, "MotorFL");
        MotorBL = hardwareMap.get(DcMotor.class, "MotorBL");
        MotorFR = hardwareMap.get(DcMotor.class, "MotorFR");
        MotorBR = hardwareMap.get(DcMotor.class, "MotorBR");
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        Elbow = hardwareMap.get(DcMotor.class, "Elbow");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        MotorFL.setDirection(DcMotor.Direction.REVERSE);
        MotorBL.setDirection(DcMotor.Direction.REVERSE);
        MotorFR.setDirection(DcMotor.Direction.FORWARD);
        MotorBR.setDirection(DcMotor.Direction.FORWARD);

        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //setArmPosition(0, 20);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Reset encoders if dpad_down is pressed
            if (gamepad2.dpad_down) {
                Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Trigger IK calculation with gamepad2.x (instead of gamepad2.square)
            if (gamepad2.x) {
                calculationIK();
            }

            if (gamepad2.dpad_up) {
                // Move to fully extended position
                setArmPosition(68.5, 0);  // Fully extend the arm
            }



            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            MotorFL.setPower(leftFrontPower);
            MotorFR.setPower(rightFrontPower);
            MotorBL.setPower(leftBackPower);
            MotorBR.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    public void calculationIK() {
        // Law of cosines to find the angle at the elbow (theta_2)
        double L1 = 33.0;  // Length of the first arm segment
        double L2 = 35.5;  // Length of the second arm segment

        // Target position
        double xTarget = 0;
        double zTarget = 34;  // Starting y (14) + 20 cm upwards

        // Calculate the distance to the target point
        double distanceToTarget = Math.sqrt(xTarget * xTarget + zTarget * zTarget);

        // Check if the point is reachable
        if (distanceToTarget > (L1 + L2)) {
            telemetry.addData("Error", "Target is out of reach.");
            telemetry.update();
        } else {
            // Law of cosines to find the angle at the elbow (theta_2)
            double cosTheta2 = (L1 * L1 + L2 * L2 - distanceToTarget * distanceToTarget) / (2 * L1 * L2);
            double theta2 = Math.acos(cosTheta2);  // Elbow angle in radians

            // Using trigonometric relationships to find the angle at the shoulder (theta_1)
            double k1 = L1 + L2 * Math.cos(theta2);
            double k2 = L2 * Math.sin(theta2);
            double theta1 = Math.atan2(zTarget, xTarget) - Math.atan2(k2, k1);  // Shoulder angle in radians

            // Convert radians to degrees
            double theta1Deg = Math.toDegrees(theta1);
            double theta2Deg = Math.toDegrees(theta2);

            // Convert degrees to ticks
            int ShoulderTargetPos = (int) (theta1Deg * 58.678);  // Adjusted ticks conversion factor
            int ElbowTargetPos = (int) (theta2Deg * 30.9576);

            // Set target positions and move motors
            Shoulder.setTargetPosition(ShoulderTargetPos);
            Elbow.setTargetPosition(ElbowTargetPos);

            Shoulder.setPower(0.3);
            Elbow.setPower(0.3);

            Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Display position and target info on telemetry
            telemetry.addData("Theta1", theta1Deg);
            telemetry.addData("Theta2", theta2Deg);
            telemetry.addData("Shoulder Target", ShoulderTargetPos);
            telemetry.addData("Elbow Target", ElbowTargetPos);
            telemetry.update();
        }
    }
    public void setArmPosition(double x, double z) {
        // Lengths of arm segments
        double L1 = 33.0;
        double L2 = 35.5;

        double distanceToTarget = Math.sqrt(x * x + z * z);

        if (distanceToTarget > (L1 + L2)) {
            telemetry.addData("Status", "Target out of reach");
        } else {
            double cosTheta2 = (L1 * L1 + L2 * L2 - distanceToTarget * distanceToTarget) / (2 * L1 * L2);
            double theta2 = Math.acos(cosTheta2);
            double k1 = L1 + L2 * Math.cos(theta2);
            double k2 = L2 * Math.sin(theta2);
            double theta1 = Math.atan2(z, x) - Math.atan2(k2, k1);

            int shoulderTargetPos = (int) Math.toDegrees(theta1) * (int) 58.678;  // Convert to ticks
            int elbowTargetPos = (int) Math.toDegrees(theta2) * (int) 30.9576;

            Shoulder.setTargetPosition(shoulderTargetPos);
            Elbow.setTargetPosition(elbowTargetPos);

            Shoulder.setPower(0.3);
            Elbow.setPower(0.3);

            Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}