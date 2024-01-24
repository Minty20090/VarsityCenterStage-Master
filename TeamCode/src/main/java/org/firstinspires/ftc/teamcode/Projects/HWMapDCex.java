package org.firstinspires.ftc.teamcode.Projects;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class HWMapDCex extends Project{
    public DcMotorEx fLeftWheel = null;
    public DcMotorEx fRightWheel = null;
    public DcMotorEx bLeftWheel = null;
    public DcMotorEx bRightWheel = null;
    public DcMotor flip = null;
    public DcMotorEx lift = null;
    public Servo clawR = null;
    public Servo clawL = null;
    public DcMotorEx ext = null;

    //public Servo wrist = null;
    //public DcMotor wrist = null;
    public BNO055IMU imu;

    public WebcamName camera = null;


    @Override
    public void init(HardwareMap hwMap) {
        // Get motors from hardware map
        fLeftWheel = hwMap.get(DcMotorEx.class, "FrontLeft");
        fRightWheel = hwMap.get(DcMotorEx.class, "FrontRight");
        bLeftWheel = hwMap.get(DcMotorEx.class, "BackLeft");
        bRightWheel = hwMap.get(DcMotorEx.class, "BackRight");
        ext = hwMap.get(DcMotorEx.class, "slide");
        lift = hwMap.get(DcMotorEx.class, "lift");
        //stick = hwMap.servo.get("Stick");
       clawL = hwMap.servo.get("clawL");
        clawR = hwMap.servo.get("clawR");
        //wrist = hwMap.servo.get("wrist");
        //wrist = hwMap.dcMotor.get("wrist");



        // Set Direction
        fRightWheel.setDirection(DcMotor.Direction.FORWARD);
        fLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        bRightWheel.setDirection(DcMotor.Direction.FORWARD);
        bLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        ext.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        //wrist.setDirection(DcMotor.Direction.FORWARD);

        // Set run mode
        fRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ext.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set brakes
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get webcam from hardware map
        camera = hwMap.get(WebcamName.class, "webcam");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag="IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Stop();
    }
    public void Stop(){
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
        ext.setPower(0);

        lift.setPower(0);

//        slide.setPower(0);
        //wrist.setPower(0);
    }

}

