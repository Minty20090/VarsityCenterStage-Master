package org.firstinspires.ftc.teamcode.Projects;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class HWMap extends Project{
    public DcMotor fLeftWheel = null;
    public DcMotor fRightWheel = null;
    public DcMotor bLeftWheel = null;
    public DcMotor bRightWheel = null;
    public DcMotor slide = null;
    public Servo rClaw = null;
    public Servo lClaw = null;
    public Servo oClaw = null;
    //public Servo wrist = null;
    //public DcMotor wrist = null;


    public WebcamName camera = null;


    @Override
    public void init(HardwareMap hwMap) {
        // Get motors from hardware map
        fLeftWheel = hwMap.dcMotor.get("fleft");
        fRightWheel = hwMap.dcMotor.get("fright");
        bLeftWheel = hwMap.dcMotor.get("bleft");
        bRightWheel = hwMap.dcMotor.get("bright");
//        slide = hwMap.dcMotor.get("slide");
//        rClaw = hwMap.servo.get("rClaw");
//        lClaw = hwMap.servo.get("lClaw");
//        oClaw = hwMap.servo.get("oClaw");
        //wrist = hwMap.servo.get("wrist");
        //wrist = hwMap.dcMotor.get("wrist");



        // Set Direction
        fRightWheel.setDirection(DcMotor.Direction.FORWARD);
        fLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        bRightWheel.setDirection(DcMotor.Direction.FORWARD);
        bLeftWheel.setDirection(DcMotor.Direction.REVERSE);
//        slide.setDirection(DcMotor.Direction.FORWARD);
        //wrist.setDirection(DcMotor.Direction.FORWARD);

        // Set run mode
        fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set brakes
        fRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get webcam from hardware map
//        camera = hwMap.get(WebcamName.class, "camera");

        Stop();
    }
    public void Stop(){
        fRightWheel.setPower(0);
        fLeftWheel.setPower(0);
        bRightWheel.setPower(0);
        bLeftWheel.setPower(0);
//        slide.setPower(0);
        //wrist.setPower(0);
    }
}