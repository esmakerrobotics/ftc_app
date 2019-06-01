package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Sample", group = "Autonomous")
public class SampleAutonomous extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + 10000);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + 10000);
        leftMotor.setPower(1);
        rightMotor.setPower(1);

        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            telemetry.addData("Position", "Left: " + leftMotor.getCurrentPosition());
            telemetry.addData("Position", "Right: " + rightMotor.getCurrentPosition());
            telemetry.update();
        }

    }
}
