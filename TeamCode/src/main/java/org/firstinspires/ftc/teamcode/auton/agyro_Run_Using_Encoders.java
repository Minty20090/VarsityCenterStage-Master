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
public class agyro_Run_Using_Encoders extends LinearOpMode {
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
    public String location = "Middle";

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



        // Side c = Side.rBlue;


        int side = 1;
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


            if (gamepad1.a) {
                telemetry.addLine("rBlue");
                telemetry.update();
                side = 1;
            }
            if (gamepad1.b) {
                telemetry.addLine("lBlue");
                telemetry.update();
                side = 2;
            }
            if (gamepad1.x) {
                telemetry.addLine("rRed");
                telemetry.update();
                side = 3;
            }
            if (gamepad1.y) {
                telemetry.addLine("lRed");
                telemetry.update();
                side = 4;
            }

            runTime.reset();
            if (side == 1 || side == 2) {
                webcam.setPipeline(BluePropDetectionPipeline);
                BluePropLocation elementLocation = BluePropDetectionPipeline.getPropLocation();
                if (elementLocation == BluePropLocation.RIGHT) {
                    telemetry.addLine("right");
                    telemetry.update();
                    location = "Right";


                } else if (elementLocation == BluePropLocation.LEFT) {
                    telemetry.addLine("left");
                    telemetry.update();
                    location = "Left";

                } else if (elementLocation == BluePropLocation.MIDDLE) {
                    telemetry.addLine("middle");
                    telemetry.update();
                    location = "Middle";


                } else {
                    telemetry.addLine("not detected");
                    telemetry.update();
                    location = "Middle";
                }
            } else {
                webcam.setPipeline(RedPropDetectionPipeline);
                RedPropLocation elementLocation = RedPropDetectionPipeline.getPropLocation();
                if (elementLocation == RedPropLocation.RIGHT) {
                    telemetry.addLine("right");
                    telemetry.update();
                    location = "Right";
                } else if (elementLocation == RedPropLocation.LEFT) {
                    telemetry.addLine("left");
                    telemetry.update();
                    location = "Left";
                } else if (elementLocation == RedPropLocation.MIDDLE) {
                    telemetry.addLine("middle");
                    telemetry.update();
                    location = "Middle";

                } else {
                    telemetry.addLine("not detected");
                    telemetry.update();
                    location = "Middle";
                }

            }


            while (opModeIsActive()) {
                location = "Right";

                robot.clawL.setPosition(0);
                sleep(500);
                robot.wrist.setPosition(1);



                if(side==1) {  // rblue
                    //Blue stage
                    spikeBRight(location);
                    driveThroughRiggingB("long");
                    if (location != "Middle") {
                        alignAprilTags(side,location);
                        break;
                    }

                }
                if(side==2){
                    //Blue back stage
                    spikeBLeft(location);
                    driveThroughRiggingB("short");
                    alignAprilTags(side,location);

                    break;

                }
                if(side == 3){
                    //Red backstage
                    spikeRRight(location);
                    driveThroughRiggingR("short");
                    alignAprilTags(side,location);
                    break;

                }
                if(side == 4) {
                    // red stage //lred
                    spikeRLeft(location);
                    driveThroughRiggingR("long");
                    if (location != "Middle") {
                        alignAprilTags(side,location);
                        break;
                    }
                    break;
                }

                break;


            }
        }
    }
    public void drop(){
        backTiles(.2);
        robot.wrist.setPosition(.25);
        sleep(1000);
        tiles(.35);
        robot.clawL.setPosition(1);
        sleep(500);
        backTiles(.3);
        robot.clawR.setPosition(1);
        robot.clawL.setPosition(0);
        sleep(500);
        robot.wrist.setPosition(1);
    }

    // BLUE LEFT
    public void spikeBLeft(String location) { // tress is to the right
        if (location == "Middle") { //DONE
            tiles(.8);
            drop();

        }
        else if(location == "Right"){
            tiles(1.1);
            turn(-65);
            backTiles(.1);
            drop();
            tiles(.35);
            sleep(1000);
            turn(75);

        }
        else if(location == "Left"){
            tiles(1);
            turn(65);
            backTiles(.35);
            drop();
            tiles(.2);
            turn(-45);


        }
    }

    // RED LEFT
    public void spikeRLeft(String location) { // tress is to the right
        if (location == "Middle") {
            tiles(.9);
            drop();

        }
        else if(location == "Right"){
            tiles(1.1);
            turn(-50);
            sleep(500);
            drop();
            tiles(.2);
            turn(-30);


        }
        else if(location == "Left"){
            tiles(1);
            turn(50);
            backTiles(.2);
            drop();
            sleep(1000);
            turn(-50);

        }
    }

    // BLUE RIGHT
    public void spikeBRight(String location) {// tress is to the left
        if (location == "Middle") {
            tiles(.8);
            drop();

        }
        else if(location == "Right"){
            tiles(1.1);
            turn(-65);
            backTiles(.25);
            drop();
            tiles(.35);
            sleep(1000);
            turn(65);

        }
        else if(location == "Left"){
            tiles(1);
            turn(50);
            backTiles(.25);
            drop();
            sleep(1000);
            turn(-50);

        }
    }
    // RIGHT RED
    public void spikeRRight(String location) {// tress is to the left // tentatively done
        if (location == "Middle") {
            tiles(.85);
            drop();
            sleep(2000);

        }
        else if(location == "Right"){
            tiles(1);
            turn(-75);
            sleep(500);
            backTiles(.2);
            drop();
            tiles(.2);
            turn(55);


        }
        else if(location == "Left"){
            tiles(1);
            turn(50);
            backTiles(.15);
            drop();
            sleep(1000);
            turn(-50);

        }
    }

    public void driveThroughRiggingR(String dist) {
        if(location == "Middle") {
            if(dist == "short"){
                tiles(.2);
                turn(-70);
                tiles(1);
            }

        }
        else if (location == "Left") {

            if(dist == "short"){
                tiles(.6);
                turn(-65);
                tiles(.8);
            }
            else if (dist == "long") {
                tiles(1);
                turn(-65);
                tiles(2.5);

            }
        }
        else { // RED SHORT SIDE
            if(dist == "short"){
                if (location == "Right") {
                    tiles(.7);
                }
                turn(-60);
                tiles(1.2);
            }
            else if (dist == "long") {
                tiles(1);
                turn(-70);
                tiles(2.5);

            }
        }

    }
    public void driveThroughRiggingB(String dist) {
        if(location == "Middle") {
            if(dist == "short"){
                tiles(.2);
                turn(65);
                tiles(1);
            }

        }
        else if(location == "Right") {

            if(dist == "short"){
                tiles(.6);
                turn(65);
                tiles(1);
            }
            else if (dist == "long") {
                tiles(1);
                turn(65);
                tiles(2.5);

            }
        }
        else if(location == "Left"){

            if(dist == "short"){
                tiles(.8);
                turn(75);
                tiles(1);
            }
            else if (dist == "long") {
                tiles(1.2);
                turn(75);
                tiles(3);

            }
        }


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

            if((side == 1 || side == 2) && (location == "Left" || location == "Right")){
                strafeLeft(200);
            }
            else if((side == 1 || side == 2) && location == "Middle") {
                strafeRight(200);
            }
            else if ((side == 3 || side == 4)){
                strafeRight(200);
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
                    strafeLeft(200);
                }
                else if (tagOfInterest.id < targetTagNum){
                    strafeRight(200);
                }
                else {
                    setALLPower(0);
                }

            }
            else if (side == 3 || side == 4) {
                if(tagOfInterest.id > targetTagNum) {
                    strafeLeft(200);
                }
                else if (tagOfInterest.id < targetTagNum){
                    strafeRight(200);
                }
                else {
                    setALLPower(0);
                }
            }

        }

        setALLPower(0);
        if(side == 3 || side == 4){
            if (location == "Left") {
                strafeRight(200);
                sleep(1000);
                setALLPower(0);
            }
            if (location == "Middle") {
                strafeRight(200);
                sleep(600);
                setALLPower(0);
            }
            if (location == "Right") {
                strafeRight(200);
                sleep(800);
                setALLPower(0);
            }
        }
        else {
            if (location == "Left") {
                strafeLeft(200);
                sleep(1000);
                setALLPower(0);
            }
            if (location == "Middle") {
                strafeLeft(400);
                sleep(500);
                setALLPower(0);
            }
        }
        tiles(.8);
        sleep(1000);
        robot.tipper.setPosition(0);
        sleep(2000);
        backTiles(.4);
        robot.tipper.setPosition(0);
        sleep(500);
        backTiles(.1);
        robot.tipper.setPosition(1);
        tiles(.2);
        strafeRight(800);
        sleep(1000);
        setALLPower(0);
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

    public void strafeRight(int power) {

        robot.fLeftWheel.setVelocity(power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(power);
    }
    public void strafeLeft(int power) {

        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(power);
        robot.bLeftWheel.setVelocity(power);
        robot.bRightWheel.setVelocity(-power);
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