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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")
@Disabled
public class TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime keyPressCheck = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armRotation = null;
    private DcMotor armLength = null;
    private Servo arm = null;
    private Servo armRotationAngle = null;
    private int oldMotorRead = 0;
    private double armMotorPowerTimes = 0.2;
    private final double armLockSpeed = 0.1;
    private final int middlePosition = 2150;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.dcMotor.get("leftMotor");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive = hardwareMap.dcMotor.get("rightMotor");
        armRotation = hardwareMap.dcMotor.get("armRotation");
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLength = hardwareMap.dcMotor.get("armLength");
        armLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm = hardwareMap.servo.get("cleanerRotation");

        double armRotationPower;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftDrive.setPower(gamepad1.left_stick_y);
            rightDrive.setPower(gamepad1.right_stick_y);

            if (keyPressCheck.milliseconds() >= 300) {
                if (gamepad1.left_bumper) {
                    keyPressCheck.reset();
                    armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sleep(200);
                }
            }

            if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_y <= -0.1) {
                if (armRotation.getCurrentPosition() > (middlePosition + 200) && -gamepad2.left_stick_y > 0) {
                    armRotationPower = -gamepad2.left_stick_y * 0.1;
                } else if(armRotation.getCurrentPosition() < (middlePosition - 200) && -gamepad2.left_stick_y < 0) {
                    armRotationPower = -gamepad2.left_stick_y * 0.1;
                } else {
                    armRotationPower = -gamepad2.left_stick_y * 0.7;
                }
            } else {
                if (armRotation.getCurrentPosition() > (middlePosition + 30)) {
                    armRotationPower = -0.2;
                } else if(armRotation.getCurrentPosition() < (middlePosition - 30)) {
                    armRotationPower = 0.2;
                } else {
                    armRotationPower = 0;
                }
            }

            if (gamepad2.right_bumper) {
                arm.setPosition(arm.getPosition() + 0.01);
            }
            if (gamepad2.left_bumper) {
                arm.setPosition(arm.getPosition() - 0.01);
            }
            armRotation.setPower(armRotationPower);
            if (armLength.getCurrentPosition() <= 0 && armLength.getCurrentPosition() >= -9300) {
                armLength.setPower(gamepad2.right_stick_y);
            } else {
                if (armLength.getCurrentPosition() >= 0) {
                    if (gamepad2.right_stick_y < 0) {
                        armLength.setPower(gamepad2.right_stick_y);
                    } else {
                        armLength.setPower(0);
                    }
                } else if (armLength.getCurrentPosition() <= -9300) {
                    if (gamepad2.right_stick_y > 0) {
                        armLength.setPower(gamepad2.right_stick_y);
                    } else {
                        armLength.setPower(0);
                    }
                } else {
                    armLength.setPower(0);
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("RotationPower", armRotationPower);
            telemetry.addData("stick read", gamepad2.left_stick_y);
            telemetry.addData("Motor read", armRotation.getCurrentPosition());
            telemetry.addData("ArmLength Motor read", armLength.getCurrentPosition());
            telemetry.update();
        }
    }
}
