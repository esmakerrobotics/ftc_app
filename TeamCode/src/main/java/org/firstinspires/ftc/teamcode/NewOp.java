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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp1", group="Linear Opmode")
public class NewOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime ResetArmMotorkeyPressCheck = new ElapsedTime();
    private ElapsedTime driverControlModeCheck = new ElapsedTime();
    private ElapsedTime driveModeSwitchCheck = new ElapsedTime();
    private ElapsedTime ArmLengthControlResetCheck = new ElapsedTime();
    private ElapsedTime servoRotationCheck = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armRotation = null;
    private DcMotor armLength = null;
    private Servo arm = null;
    private Servo armRotationAngle = null;
    private int oldMotorRead = 0;
    private int driveMode = 0;
    private int driverControlMode = 0;
    private double armMotorPowerTimes = 0.2;
    private final double armLockSpeed = 0.1;
    private final int middlePosition = 2250;

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
        armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm = hardwareMap.servo.get("cleanerRotation");

        double armRotationPower;
        double leftPower, rightPower = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Set Driving Power
            if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) {
                //gp2 has priority
                double lPower, rPower = 0;
                double powerCoefficient = 1;
                powerCoefficient = gamepad2.right_trigger;
                //Calculate powers
                int movePower = (gamepad2.dpad_up ? 1:0) - (gamepad2.dpad_down ? 1:0);
                movePower *= -1;
                int turnPower = (gamepad2.dpad_left ? 1:0) - (gamepad2.dpad_right ? 1:0);
                turnPower *= -1;
                lPower = Range.clip(movePower - turnPower, -1, 1);
                rPower = Range.clip(movePower + turnPower, -1, 1);
                lPower *= powerCoefficient;
                rPower *= powerCoefficient;

                leftDrive.setPower(lPower);
                rightDrive.setPower(rPower);
            } else {
                //power give back to gp1
                if (driveMode == 0) {
                    leftDrive.setPower(gamepad1.left_stick_y);
                    rightDrive.setPower(gamepad1.right_stick_y);
                } else if (driveMode == 1) {
                    double lPower = Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1);
                    double rPower = Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -1, 1);
                    if (driverControlMode == 1) {
                        lPower *= gamepad1.right_trigger;
                        rPower *= gamepad1.right_trigger;
                    }
                    leftDrive.setPower(lPower);
                    rightDrive.setPower(rPower);
                }
            }


            //Change Driving Mode (gp1, LB)
            if (driveModeSwitchCheck.milliseconds() >= 300) {
                if (gamepad1.left_bumper) {
                    driveModeSwitchCheck.reset();
                    if (driveMode == 0) {
                        driveMode = 1;
                    } else {
                        driveMode = 0;
                    }
                }
            }

            //Change Driver Control Mode
            if (driverControlModeCheck.milliseconds() >= 300) {
                if (gamepad1.right_bumper) {
                    driverControlModeCheck.reset();
                    if (driverControlMode == 0) {
                        driverControlMode = 1;
                    } else {
                        driverControlMode = 0;
                    }
                }
            }

            //Reset Arm Rotation Motor Encoder (gp2, x)
            if (ResetArmMotorkeyPressCheck.milliseconds() >= 300) {
                if (gamepad2.x) {
                    ResetArmMotorkeyPressCheck.reset();
                    armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sleep(200);
                }
            }

            //Arm Speed Control, Temporarily pause using gp2, LB
            if ((gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_y <= -0.1) || gamepad2.left_bumper) {
                if (armRotation.getCurrentPosition() > (middlePosition + 200) && -gamepad2.left_stick_y > 0 && !gamepad2.left_bumper) {
                    armRotationPower = -gamepad2.left_stick_y * 0.1;
                } else if(armRotation.getCurrentPosition() < (middlePosition - 200) && -gamepad2.left_stick_y < 0 && !gamepad2.left_bumper) {
                    armRotationPower = -gamepad2.left_stick_y * 0.1;
                } else {
                    //when LT is pressed under 20%, activate speed control
                    if (gamepad2.left_trigger >= 0.2) {
                        armRotationPower = -gamepad2.left_stick_y * gamepad2.left_trigger;
                    } else {
                        armRotationPower = -gamepad2.left_stick_y * 0.7;
                    }
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
            armRotation.setPower(armRotationPower);

            //Set Servo Position
            if (servoRotationCheck.milliseconds() >= 20) {
                if (gamepad2.a || gamepad1.y) {
                    servoRotationCheck.reset();
                    arm.setPosition(arm.getPosition() + 0.01);
                }
            }
            if (servoRotationCheck.milliseconds() >= 20) {
                if (gamepad2.y || gamepad1.a) {
                    servoRotationCheck.reset();
                    arm.setPosition(arm.getPosition() - 0.01);
                }
            }

            //Limit Length Control Motor Position, pause using gp2, RB
            if ((armLength.getCurrentPosition() <= 0 && armLength.getCurrentPosition() >= -9300) || gamepad2.right_bumper) {
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

            //Arm Length Control Motor Reset
            if (ArmLengthControlResetCheck.milliseconds() >= 300) {
                if (gamepad2.b) {
                    ArmLengthControlResetCheck.reset();
                    armLength.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("drivingMode", driveMode);
            telemetry.addData("ControlMode", driverControlMode);
            telemetry.addData("RotationPower", armRotationPower);
            telemetry.addData("Motor read", armRotation.getCurrentPosition());
            telemetry.addData("ArmLength Motor read", armLength.getCurrentPosition());
            telemetry.update();
        }
    }
}
