package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.RedPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.BluePropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.BluePropDetectionPipeline.BluePropLocation;
import org.firstinspires.ftc.teamcode.Vision.RedPropDetectionPipeline.RedPropLocation;

import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Projects.HWMap;

@Autonomous
public class OpenCV extends LinearOpMode{
    // public HWMap robot = new HWMap();
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
    int Left = 1;
    int Middle = 2;
    int Right = 3;
    AprilTagDetection tagOfInterest = null;

    RedPropDetectionPipeline RedPropDetectionPipeline = new RedPropDetectionPipeline(telemetry);
    BluePropDetectionPipeline BluePropDetectionPipeline = new BluePropDetectionPipeline(telemetry);
    AprilTagDetectionPipeline AprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    boolean propInRange = false;
    public ElapsedTime runTime = new ElapsedTime(); //sets up a timer in the program

    @Override
    public void runOpMode() {
        // robot.init(hardwareMap);

        Side c = Side.rBlue;
        int side = 1;
        if(gamepad1.right_bumper == true){
            if(side<4) {
                side++;
            }
            else if(side == 4){
                side = 1;
            }
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);


        webcam.setPipeline(AprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        ArrayList<AprilTagDetection> currentDetections = AprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)

                if(tag.id == Left  || tag.id == Middle || tag.id == Right )
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest\nlol, get better:(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen in the history of this run, the records must be incomplete)");
            }
            else
            {
                telemetry.addLine("\nBut we thankfully HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        telemetry.update();
        sleep(20);


        waitForStart();
        switch(side) {
            case 1:

                c = Side.rBlue;

                break;
            case 2:

                c = Side.lBlue;

                break;
            case 3:

                c = Side.rRed;

                break;
            case 4:

                c = Side.lRed;

                break;
        }


        //NEEDS TO BE FIXED
        // DRIVE TO AND LINE UP WITH POLE
        runTime.reset();
        while (propInRange == false) {
            if (c == Side.rBlue || c == Side.lBlue){
                BluePropDetectionPipeline.BluePropLocation elementLocation = BluePropDetectionPipeline.getPropLocation();
                webcam.setPipeline(BluePropDetectionPipeline);
//                if (elementLocation == BluePropLocation.RIGHT) {
//                    encoderDrive(0.25, -25, 25, -25, 25);
//                    stop(1000);
//                } else if (elementLocation == BluePropLocation.LEFT) {
//                    encoderDrive(0.25, 25, -25, 25, -25);
//                    stop(1000);
//                } else if (elementLocation == BluePropLocation.MIDDLE) {
//                    encoderDrive(0.25, 25, 25, 25, 25);
//                    stop(1000);
//                } else if (elementLocation == BluePropLocation.CLOSE) {
//                    stop(1000);
//                    propInRange = true;
//                } else {
//                    encoderDrive(0.25, -25, -25, -25, -25);
//                    stop(1000);
//                }
            }
            else{
                RedPropDetectionPipeline.RedPropLocation elementLocation = RedPropDetectionPipeline.getPropLocation();
                webcam.setPipeline(RedPropDetectionPipeline);
//                if (elementLocation == RedPropLocation.RIGHT) {
//                    encoderDrive(0.25, -25, 25, -25, 25);
//                    stop(1000);
//                } else if (elementLocation == RedPropLocation.LEFT) {
//                    encoderDrive(0.25, 25, -25, 25, -25);
//                    stop(1000);
//                } else if (elementLocation == RedPropLocation.MIDDLE) {
//                    encoderDrive(0.25, 25, 25, 25, 25);
//                    stop(1000);
//                } else if (elementLocation == RedPropLocation.CLOSE) {
//                    stop(1000);
//                    propInRange = true;
//                } else {
//                    encoderDrive(0.25, -25, -25, -25, -25);
//                    stop(1000);
//                }
            }

        }


        if (propInRange == true) {
            encoderDrive(0.25, -25, 25, -25, 25);
            stop(2000);

        }




        while (opModeIsActive()) {
            sleep(20);
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
//    public void moveRobot(int time) {
//        robot.fLeftWheel.setPower(1);
//        robot.fRightWheel.setPower(1);
//        robot.bLeftWheel.setPower(1);
//        robot.bRightWheel.setPower(1);
//        sleep(time);
//        robot.fLeftWheel.setPower(0);
//        robot.fRightWheel.setPower(0);
//        robot.bLeftWheel.setPower(0);
//        robot.bRightWheel.setPower(0);
//    }
//    public void turnRobot(String direction, int degrees) {
//        if (direction == "right") {
//            robot.fRightWheel.setPower(-.5);
//            robot.bRightWheel.setPower(-.5);
//            robot.fLeftWheel.setPower(.5);
//            robot.bLeftWheel.setPower(.5);
//            sleep(degrees/45*500);
//            robot.fRightWheel.setPower(0);
//            robot.fLeftWheel.setPower(0);
//            robot.bRightWheel.setPower(0);
//            robot.bLeftWheel.setPower(0);
//        }
//        if (direction == "left") {
//            robot.fRightWheel.setPower(.5);
//            robot.bRightWheel.setPower(.5);
//            robot.fLeftWheel.setPower(-.5);
//            robot.bLeftWheel.setPower(-.5);
//            sleep(degrees/45*650);
//            robot.fRightWheel.setPower(0);
//            robot.fLeftWheel.setPower(0);
//            robot.bRightWheel.setPower(0);
//            robot.bLeftWheel.setPower(0);
//        }
//    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

}
