package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;
import org.firstinspires.ftc.teamcode.auton.BluePropDetectionPipeline.BluePropLocation;
import org.firstinspires.ftc.teamcode.auton.RedPropDetectionPipeline.RedPropLocation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BasicOpenCV extends LinearOpMode{
     public HWMapBasic robot = new HWMapBasic();
    OpenCvCamera webcam;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

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
        robot.fRightWheel.setTargetPosition(0);
        robot.fLeftWheel.setTargetPosition(0);
        robot.bRightWheel.setTargetPosition(0);
        robot.bLeftWheel.setTargetPosition(0);
        robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.gate.setPosition(0);

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

                robot.fLeftWheel.setPower(.8);
                robot.fRightWheel.setPower(.8);
                robot.bLeftWheel.setPower(.8);
                robot.bRightWheel.setPower(.8);

            // START COMMETNED OUT SECTION
                sleep(20);
                if(side==1) {
                    //Blue stage

                    spikeB(location,side);
                    robot.gate.setPosition(0);
//                    turn(105,.8);
//                    sleep(1000);
//                    tiles(3.5);
                    break;
                }
                if(side==2){
                    //Blue back tage
                    spikeB(location,side);
                    robot.gate.setPosition(0);
//                    turn(100,.8);
//                    sleep(1000);
//                    tiles(1.5);
                    break;

                }
                if(side == 3){
                    //Red backstage
                    spikeR(location,side);
//                    robot.gate.setPosition(0);
//                    turn(100,-.8);
//                    sleep(1000);
//                    tiles(1.5);
                    break;

                }
                if(side == 4) {
                    //Red stage - Far
                    spikeR(location,side);
                    robot.gate.setPosition(0);
//                    sleep(1000);
//                    turn(105,-.8);
//                    sleep(1000);
//                    tiles(3.5);
                    break;
                }
                break;

// END COMMETNED OUT SECTION
            }


        }
    }
    public void drop(){
        robot.gate.setPosition(1);
        sleep(1000);
    }
    public void spikeB(String location, int side) { // blue
        if (location == "Middle") {
            System.out.println("bet");
            tiles(1.2,side);
            drop();
            tiles(-1.1,side);

        }
        else if(location == "Right"){
            tiles(1.1,side);
            turn(95,-.8);
            tiles(.2,side);
            drop();
            tiles(-.25,side);
            turn(95,.8);
            tiles(-1,side);
        }
        else if(location == "Left"){
            tiles(1.1,side);
            turn(95,.8);
            tiles(.2,side);
            drop();
            tiles(-.25,side);
            turn(95,-.8);
            tiles(-1,side);


        }
    }
    public void spikeR(String location, int side) {
        if (location == "Middle") {
            System.out.println("bet");
            tiles(1.2,side);
            drop();
            tiles(-1.1,side);

        }
        else if(location == "Right"){
            tiles(1.1,side);
            turn(100,-.8);
            tiles(.2,side);
            drop();
            tiles(-.25,side);
            turn(95,.8);
            tiles(-1,side);

        }
        else if(location == "Left"){
            tiles(1.1,side);
            turn(95,.8);
            tiles(.2,side);
            drop();
            tiles(-.25,side);
            turn(95,-.8);
            tiles(-1,side);
//
        }
    }
    public void tiles(double tiles, int side){
        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();
        robot.fLeftWheel.setPower(.5);
        robot.fRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * -580));
        robot.fRightWheel.setTargetPosition((int)(fright + tiles * -580));
        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * -580));
        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * -580));
        sleep(2000);
//        if (tiles > 0 && (side == 3 || side == 4)) {
//            correction(tiles);
//        }


    }
    public void correction( double tiles) {
        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();
        robot.fLeftWheel.setPower(.5);
        robot.fRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * -70));
        robot.fRightWheel.setTargetPosition((int)(fright + tiles * 70));
        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * 70));
        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * -70));
        sleep(500);
    }


    public void turn(int degrees, double direction){
        String turn;

        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();
        if (direction < 0) {
            turn = "right";
        }
        else {
            turn = "left";
        }

        if (turn == "left") {
            robot.bLeftWheel.setTargetPosition((int) (bleft + (int)(degrees) * (int) (287/90)));
            robot.fLeftWheel.setTargetPosition((int) (fleft + (int)(degrees) * (int) (369/90)));
            robot.bRightWheel.setTargetPosition((int) (bright+ (int) (degrees * (int) (-279/90))));
            robot.fRightWheel.setTargetPosition((int) (fright + (int) (degrees * (int) (-367/90))));

        }
        if (turn == "right") {

            robot.bLeftWheel.setTargetPosition((int) (bleft + (int)(degrees * (int)(-290/90))));
            robot.fLeftWheel.setTargetPosition((int) (fleft + (int)(degrees * (int)(-390/90))));
            robot.bRightWheel.setTargetPosition((int) (bright+ (int)(degrees * (int)(285/90))));
            robot.fRightWheel.setTargetPosition((int) (fright + (int)(degrees * (int)(340/90))));
        }

        robot.fLeftWheel.setPower(.8);
        robot.fRightWheel.setPower(.8);
        robot.bLeftWheel.setPower(.8);
        robot.bRightWheel.setPower(.8);
        sleep(2000);
    }



    //encoder method
    public void encoderDrive(double speed,
                             double frontLeftCounts, double frontRightCounts, double backLeftCounts, double backRightCounts) {
        int newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newFrontLeftTarget = robot.fLeftWheel.getCurrentPosition() + (int) (frontLeftCounts);
//            newFrontRightTarget = robot.fRightWheel.getCurrentPosition() + (int) (frontRightCounts);
//            newBackLeftTarget = robot.bLeftWheel.getCurrentPosition() + (int) (backLeftCounts);
//            newBackRightTarget = robot.bRightWheel.getCurrentPosition() + (int) (backRightCounts);
//            robot.fLeftWheel.setTargetPosition(newFrontLeftTarget);
//            robot.fRightWheel.setTargetPosition(newFrontRightTarget);
//            robot.bLeftWheel.setTargetPosition(newBackLeftTarget);
//            robot.bRightWheel.setTargetPosition(newBackRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.fRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.bRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.fLeftWheel.setPower(Math.abs(speed));
//            robot.fRightWheel.setPower(Math.abs(speed));
//            robot.bLeftWheel.setPower(Math.abs(speed));
//            robot.bRightWheel.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (robot.fLeftWheel.isBusy() && robot.fRightWheel.isBusy() && robot.bLeftWheel.isBusy() && robot.bRightWheel.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d");
//
//                telemetry.update();
//            }
//
//            // Stop all motion;
//
//
//            // Turn off RUN_TO_POSITION

        }
    }
    public void stop(int time) {

        sleep(time);
   }

}