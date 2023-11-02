package org.firstinspires.ftc.teamcode.Projects;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class FleaFlickerMap extends Project{
    public DcMotor fLeftWheel = null;
    public DcMotor fRightWheel = null;
    public DcMotor bLeftWheel = null;
    public DcMotor bRightWheel = null;
    public DcMotor outtakeLift = null;
    public DcMotor intakeLift = null;
    public Servo gate = null;
    public Servo clawR = null;
    public Servo clawL = null;
    public Servo intakeR = null;
    public Servo intakeL = null;

    public WebcamName camera = null;


    @Override
    public void init(HardwareMap hwMap) {
        // Get motors from hardware map
        fLeftWheel = hwMap.dcMotor.get("FrontLeft");
        fRightWheel = hwMap.dcMotor.get("FrontRight");
        bLeftWheel = hwMap.dcMotor.get("BackLeft");
        bRightWheel = hwMap.dcMotor.get("BackRight");
        outtakeLift = hwMap.dcMotor.get("outtakeMotor");
        intakeLift = hwMap.dcMotor.get("intakeMotor");
//        gate = hwMap.servo.get("gate");
        clawR = hwMap.servo.get("clawR");
        clawL = hwMap.servo.get("clawL");
//        intakeR = hwMap.servo.get("intakeR");
//        intakeL = hwMap.servo.get("intakeL");




        // Set Direction
        fRightWheel.setDirection(DcMotor.Direction.FORWARD);
        fLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        bRightWheel.setDirection(DcMotor.Direction.FORWARD);
        bLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        outtakeLift.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);

        // Set run mode
        fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brakes
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get webcam from hardware map
        camera = hwMap.get(WebcamName.class, "camera");



        Stop();
    }
    public void Stop(){
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
        outtakeLift.setPower(0);
        intakeLift.setPower(0);
    }
}
