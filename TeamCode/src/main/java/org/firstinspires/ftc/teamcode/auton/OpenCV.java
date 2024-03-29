package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.auton.BluePropDetectionPipeline.BluePropLocation;
import org.firstinspires.ftc.teamcode.auton.RedPropDetectionPipeline.RedPropLocation;

@Autonomous
public class OpenCV extends LinearOpMode{
     public HWMap robot = new HWMap();

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
        robot.lift.setTargetPosition(0);
        robot.ext.setTargetPosition(0);
        robot.fRightWheel.setTargetPosition(0);
        robot.fLeftWheel.setTargetPosition(0);
        robot.bRightWheel.setTargetPosition(0);
        robot.bLeftWheel.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Side c = Side.rBlue;
        int side = 1;
        if (gamepad1.right_bumper == true) {
            if (side < 4) {
                side++;
            } else if (side == 4) {
                side = 1;
            }
        }
        switch (side) {
            case 1:
                telemetry.addLine("rBlue");
                telemetry.update();
                break;
            case 2:
                telemetry.addLine("lBlue");
                telemetry.update();
                break;
            case 3:
                telemetry.addLine("rRed");
                telemetry.update();
                break;
            case 4:
                telemetry.addLine("lRed");
                telemetry.update();
                break;
        }

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

            runTime.reset();
            if (side == 1 || side == 2) {
                webcam.setPipeline(BluePropDetectionPipeline);
                BluePropDetectionPipeline.BluePropLocation elementLocation = BluePropDetectionPipeline.getPropLocation();
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
                RedPropDetectionPipeline.RedPropLocation elementLocation = RedPropDetectionPipeline.getPropLocation();
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

                }

            }


            while (opModeIsActive()) {
                robot.lift.setPower(.5);
                sleep(1000);
                robot.lift.setPower(0);
                sleep(2000);
                robot.lift.setPower(-.5);
                sleep(999);
                robot.lift.setPower(0);
                sleep(20);
                if(side==1) {
                    //Blue backstage
                    spikeB(location);
                    tiles(-.75);
                    turn(90,.8);
                    tiles(2);
                }
                if(side==2){
                    //Blue Stage
                    spikeB(location);
                    tiles(-1);
                    turn(90,.8);
                    tiles(1);
                    turn(90,-.8);
                    tiles(2);
                    turn(90,.8);
                    tiles(3);

                }
                if(side==3){
                    //Red backstage
                    spikeR(location);
                    tiles(-.75);
                    turn(90,-.8);
                    tiles(2);
                }
                else {
                    //Red stage - Far
                    spikeR(location);
                    tiles(-1);
                    turn(90,-.8);
                    tiles(1);
                    turn(90,.8);
                    tiles(2);
                    turn(90,-.8);
                    tiles(3);
                }


            }


        }
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
            robot.fLeftWheel.setTargetPosition((fleft));
            robot.fRightWheel.setTargetPosition((int) (fright + (degrees/90 * 564)));
            robot.bLeftWheel.setTargetPosition((int) (bleft + (degrees/90 * -351)));
            robot.bRightWheel.setTargetPosition((int) (bright+ degrees/90 * 201));
        }
        if (turn == "right") {
            robot.fLeftWheel.setTargetPosition((int) (fleft + (degrees/90 * -11)));
            robot.fRightWheel.setTargetPosition((int) (fright + (degrees/90 * -562)));
            robot.bLeftWheel.setTargetPosition((int) (bleft + (degrees/90 * 300)));
            robot.bRightWheel.setTargetPosition((int) (bright+ (degrees/90 * -233)));
        }

        robot.fLeftWheel.setPower(.8);
        robot.fRightWheel.setPower(.8);
        robot.bLeftWheel.setPower(.8);
        robot.bRightWheel.setPower(.8);
    }


    public void tiles(double tiles){
        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();

        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * 480));
        robot.fRightWheel.setTargetPosition((int)(fright + tiles * 480));
        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * 600));
        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * 600));
        robot.fLeftWheel.setPower(.8);
        robot.fRightWheel.setPower(.8);
        robot.bLeftWheel.setPower(.8);
        robot.bRightWheel.setPower(.8);
    }

    public void drop(){
       robot.clawR.setPosition(1);
       robot.lift.setTargetPosition(500);
       robot.clawR.setPosition(0);

    }
    public void spikeB(String location) { // blue
        if (location == "Middle") {
            System.out.println("bet");
            tiles(1);
            sleep(500);
            drop();
            sleep(2000);
        }
        else if(location == "Right"){
            tiles(1);
            turn(90,-.8);
            sleep(500);
            drop();
            sleep(2000);
            turn(90,.8);
        }
        else if(location == "Left"){
           tiles(1);
            turn(90,.8);
            sleep(500);
            drop();
            sleep(2000);
            turn(90,-.8);


        }
    }
    public void spikeR(String location) {
        if (location == "Middle") {
            System.out.println("bet");
            tiles(1);
            sleep(500);
            drop();



        }
        else if(location == "Right"){
            tiles(1);
            turn(90,.8);
            sleep(500);
            drop();
            turn(90,-.8);

        }
        else if(location == "Left"){
            tiles(1);
            turn(90,.8);
            sleep(500);
            drop();
            turn(90,-.8);

        }
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