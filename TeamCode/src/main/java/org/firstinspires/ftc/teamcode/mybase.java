package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraCalibration;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


 public class mybase {
    private OpMode myOpMode;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    /**
     * Dc Motors
     */
    public DcMotor motor[] = new DcMotor[MOTOR_NUMBER];

    double Kp = 0.015; /*increase the Kp to enhance the oscillation and reduce the adjusting time *///0.008,0.0024,0.005
    double Ki = 90000;
    public BNO055IMU imu;
    Orientation angles;
    boolean IsRunning;
    double sum;
    double errorsum;
public double a;

    private DcMotor armRotation = null;
    private DcMotor armLength = null;
    private Servo arm = null;

    private static final byte leftDrive = 0;
    private static final byte rightDrive = 1;
    private static final byte MOTOR_NUMBER = 2;
    private static final String VUFORIA_KEY = "AWZ1FLb/////AAAAGU5Ty2dHZETahEyraiiGxnEQecKVoS80GUybCIaO/G9VaSocMoYwajkfThEQAnKSjRKBKImxk4y6DQ/rwvaiEHEV3Zw5gFDJrRzwioETWkY6VxHxto9LcE8kyU+gek6uVaUKbYHMWijhGvvlzQ+XgeRbjssMmE7/usViVHtUKquiE+pgRXK5l6N872+b9Rfgj++crhShbWGDZ5pY3amumchaeDWh8hpBKkWvEgAfa5SH6gkomgK28q2gd8ibiRIwEvEY8vQCiv+wzpfDFfz2qHyjGNEKh7rV4eTz3kf3mBmotu37UZzTyUFCsqXJo/LJTRrLPY9pxuQauenDF0s/T1dYxZeVS0cbMBQ6JpssuqYw";

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    public mybase() {/* Constructor */}

     private double power[] = new double[MOTOR_NUMBER];

    public void initializeRobot(OpMode OP_MODE) {

        // Save reference to Hardware map
        myOpMode = OP_MODE;
        motor[leftDrive] = myOpMode.hardwareMap.dcMotor.get("leftMotor");
        motor[rightDrive] = myOpMode.hardwareMap.dcMotor.get("rightMotor");

        motor[leftDrive].setDirection(DcMotor.Direction.FORWARD);
        motor[rightDrive].setDirection(DcMotor.Direction.REVERSE);
        armRotation = myOpMode.hardwareMap.dcMotor.get("armRotation");
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLength =  myOpMode.hardwareMap.dcMotor.get("armLength");
        armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm =  myOpMode.hardwareMap.servo.get("cleanerRotation");

            for (int i = 0; i < MOTOR_NUMBER; i++)
                power[i] = 0.0;


        double armRotationPower;
        double leftPower, rightPower = 0;

        setDcMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        initVuforia();
        initTfod();


    }


     public void setMotorPower(double LFPower, double RFPower, double LBPower, double RBPower /*,double CPower*/) {
         double maxPower = Math.max(Math.max(abs(LFPower), abs(LBPower)), Math.max(abs(RFPower), abs(RBPower)));

         if (maxPower >1) {
             LFPower = LFPower / maxPower * 1;
             RFPower = RFPower / maxPower *1;

         }

         power[leftDrive] = Range.clip(LFPower, -1, 1);
         power[rightDrive] = Range.clip(RFPower, -1, 1);


         for (int i = 0; i < MOTOR_NUMBER; i++)
             motor[i].setPower(power[i]);

     }
     void setMotorPower(double L_POWER1, double R_POWER1)
     {
         setMotorPower(L_POWER1, R_POWER1, L_POWER1, R_POWER1);
    }


    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */

    private void initTfod() {
        int tfodMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void detection() {

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            myOpMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        myOpMode.telemetry.addData(">", "Press Play to start tracking");
        myOpMode.telemetry.update();

        if (IsRunning == true) {
            if (tfod != null) {
                tfod.activate();
            }

            while (IsRunning == true) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        myOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (silverMineral1X != -1 && silverMineral2X != -1) {
                                myOpMode.telemetry.addData("Gold Mineral Position", "right");


                                IMUturn(20);
                                sleep(100);
                                tfod.deactivate();
                                setMotorPower(0.8,0.8);
                                sleep(500);
                                setMotorPower(0.5,0.5);
                                sleep(300);
                                IMUturn(90);
                                setMotorPower(0.8,0.8);
                                sleep(1000);
                                IMUturn(135);
                                setMotorPower(-0.8,-0.8);
                                sleep(1500);
                                armRotation.setPower(-0.5);
                                sleep(1500);
                                setMotorPower(0.8,0.8);
                                sleep(1500);
                                setMotorPower(0,0);
                            }
                            else if (goldMineralX != -1 && silverMineral1X != -1) {
                                if (goldMineralX >silverMineral1X) {
                                    myOpMode.telemetry.addData("Gold Mineral Position", "center");

                                    IMUturn(-20);
                                    sleep(100);
                                    tfod.deactivate();
                                    setMotorPower(0.8,0.8);
                                    sleep(500);
                                    setMotorPower(0.5,0.5);
                                    sleep(300);
                                    IMUturn(90);
                                    setMotorPower(0.8,0.8);
                                    sleep(1000);
                                    IMUturn(135);
                                    setMotorPower(-0.8,-0.8);
                                    sleep(1500);
                                    armRotation.setPower(-0.5);
                                    sleep(1500);
                                    setMotorPower(0.8,0.8);
                                    sleep(1500);
                                    setMotorPower(0,0);


                                } else if (goldMineralX < silverMineral1X) {
                                    myOpMode.telemetry.addData("Gold Mineral Position", "left");

                                    IMUturn(20);
                                    sleep(100);
                                    tfod.deactivate();
                                    setMotorPower(0.8,0.8);
                                    sleep(500);
                                    setMotorPower(0.5,0.5);
                                    sleep(300);
                                    IMUturn(90);
                                    setMotorPower(0.8,0.8);
                                    sleep(1000);
                                    IMUturn(135);
                                    setMotorPower(-0.8,-0.8);
                                    sleep(1500);
                                    armRotation.setPower(-0.5);
                                    sleep(1500);
                                    setMotorPower(0.8,0.8);
                                    sleep(1500);
                                   setMotorPower(0,0);


                                }
                            }

                        }
                        myOpMode.telemetry.update();
                    }
                }

            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
    }


    public void setDcMotorMode(DcMotor.RunMode mode) {
        for (int i = 0; i < MOTOR_NUMBER; i++)
            motor[i].setMode(mode);
    }




    public double getangle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


    void IMUturn(double angle) {

        if (getangle() < angle && IsRunning) {
            while (getangle() < angle && IsRunning) {
                double x = getangle();
                double error = angle - x;
                errorsum = errorsum + error;
                double power = ((abs(angle - x)) * Kp) + (errorsum / Ki);
               setMotorPower(-power,+power);

            }
        } else if (getangle() > angle && IsRunning) {
            while (getangle() > angle && IsRunning) {
                double x = getangle();
                double error = angle - x;
                errorsum = errorsum + error;
                double power = ((abs(angle - x)) * Kp) + (errorsum / Ki);
                setMotorPower(+power,-power);
            }
        }
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}