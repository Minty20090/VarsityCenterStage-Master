package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.auton.AprilTagAutonomousInitDetectionExample.FEET_PER_METER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.Projects.HWMapDCex;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.auton.BluePropDetectionPipeline.BluePropLocation;
import org.firstinspires.ftc.teamcode.auton.RedPropDetectionPipeline.RedPropLocation;

import java.util.ArrayList;

@Autonomous
public class BasicOpenCV extends LinearOpMode {
    public HWMapDCex robot = new HWMapDCex();
    int noU = 1000;
    OpenCvCamera webcam;
    AprilTagDetection tagOfInterest = null;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    // UNITS ARE METERS
    double tagsize = 0.166;
    public String location = "Left";

    RedPropDetectionPipeline RedPropDetectionPipeline = new RedPropDetectionPipeline(telemetry);
    BluePropDetectionPipeline BluePropDetectionPipeline = new BluePropDetectionPipeline(telemetry);
    AprilTagDetectionPipeline AprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    boolean propInRange = false;
    public ElapsedTime runTime = new ElapsedTime(); //sets up a timer in the program


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.lift.setTargetPosition(0);

        robot.lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        int side = 3;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {

            webcam.setPipeline(AprilTagDetectionPipeline);
            while (opModeIsActive()) {
                alignAprilTags(side, "Middle");

            }

        }
    }


    public void tiles(double tiles){
        int power = 400;
        robot.fLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts fl", robot.fLeftWheel.getCurrentPosition());
        robot.fRightWheel.setVelocity(power);
        telemetry.addData("encoder counts fr", robot.fRightWheel.getCurrentPosition());
        robot.bLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts bl", robot.bLeftWheel.getCurrentPosition());
        robot.bRightWheel.setVelocity(power);
        telemetry.addData("encoder counts br", robot.bRightWheel.getCurrentPosition());
        sleep((int) (1700*tiles));
        robot.bRightWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.fLeftWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);

    }
    public void backTiles(double tiles) {
        int power = 400;
        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(-power);
        sleep((int) (1700*tiles));
        robot.bRightWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.fLeftWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);

    }

    public void strafeRight() {
        int power = 200;
        robot.fLeftWheel.setVelocity(power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(power);
    }
    public void strafeLeft() {
        int power = 200;
        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(power);
        robot.bLeftWheel.setVelocity(power);
        robot.bRightWheel.setVelocity(-power);
    }

    public void turnRight() {
        int power = 400;
        robot.fLeftWheel.setVelocity(power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(power);
        robot.bRightWheel.setVelocity(-power);
    }
    public void turnLeft() {
        int power = 400;
        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(power);
    }

    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;

    }
    public void turn(double degrees) {
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 1) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setALLPower(motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);
    }


    public void setALLPower(double power) {
        double powerLevel = power * 500;
        robot.fRightWheel.setVelocity(powerLevel);
        robot.fLeftWheel.setVelocity(-powerLevel);
        robot.bRightWheel.setVelocity(powerLevel);
        robot.bLeftWheel.setVelocity(-powerLevel);
    }



    public void alignAprilTags(int side, String location) {
        int targetTagNum = 1;
        if(side == 1 || side == 2) {
            if (location == "Middle") {
                targetTagNum = 2;
            }
            else if (location == "Left") {
                targetTagNum = 1;
            }
            else if (location == "Right") {
                    targetTagNum = 3;
            }
        }
        else {
            if (location == "Middle") {
                targetTagNum = 5;
            }
            else if (location == "Left") {
                targetTagNum = 4;
            }
            else if (location == "Right") {
                targetTagNum = 6;
            }
        }
        webcam.setPipeline(AprilTagDetectionPipeline);

        ArrayList<AprilTagDetection> currentDetections;

        while(tagOfInterest == null) {
            currentDetections = AprilTagDetectionPipeline.getLatestDetections();

            if((side == 1 || side == 2) && location == "Left" || location == "Right"){
                strafeLeft();
            }
            else if((side == 1 || side == 2) && location == "Middle") {
                strafeRight();
            }
            else if ((side == 3 || side == 4) && location == "Left" || location == "Right"){
                strafeRight();
            }
            else if ((side == 3 || side == 4) && location == "Middle"){
                strafeRight();
            }


            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections)

                    if (tag != null) {
                        tagOfInterest = tag;
                        tagToTelemetry(tagOfInterest, targetTagNum);
                        break;
                    }
            }
        }
        setALLPower(0);
        sleep(1000);

        while(tagOfInterest.id != targetTagNum) {
            currentDetections = AprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections)

                    if (tag != null) {
                        tagOfInterest = tag;
                        tagToTelemetry(tagOfInterest, targetTagNum);
                        break;
                    }
            }
            if (side == 1 || side == 2) {
                if(tagOfInterest.id > targetTagNum) {
                    strafeLeft();
                }
                else if (tagOfInterest.id < targetTagNum){
                    strafeRight();
                }
                else {
                    setALLPower(0);
                }

            }
            else if (side == 3 || side == 4) {
                if(tagOfInterest.id > targetTagNum) {
                    strafeLeft();
                }
                else if (tagOfInterest.id < targetTagNum){
                    strafeRight();
                }
                else {
                    setALLPower(0);
                }
            }

        }
        setALLPower(0);
        robot.tipper.setPosition(0);
        sleep(1000);
        robot.tipper.setPosition(1);
    }
    void tagToTelemetry(AprilTagDetection detection, int targetTag)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
        telemetry.addLine(String.format("Target April Tag: " + targetTag));
        telemetry.update();
    }


}