/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "camera")
public class AprilTagAutonomousInitDetectionExample<tagOfInterest> extends LinearOpMode
{

    enum Side{
        Right,
        Left
    }
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    private HWMap robot = new HWMap();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.05;


    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {

        Side a = Side.Right;
        boolean isRight = true;

        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested())
        {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);



            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)

                    if(tag != null) {
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
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // align with backboard based on tag
        // There are three tags on the backboard, one on the left, one in the middle, and one on the right
        // Assuming the robot is generally in front of the backboard, the robot will move to the left or right to align with the middle tag
        // the middle tag ID is either 2 (if the set includes ID 1 or 3) or 5 (if the set includes ID 4 or 6)
        // If the robot can see the middle tag, it will move to align with it (minimize the absolute value of the x value)
        // If the robot cannot see the middle tag, but can see the left tag, it will move to the right until it can see the middle tag
        // If the robot cannot see the middle tag, but can see the right tag, it will move to the left until it can see the middle tag

        // Implementation:

        if(tagOfInterest != null)
        {
            if(tagOfInterest.id == 2 || tagOfInterest.id == 5)
            {
                // telemetry
                tagToTelemetry(tagOfInterest);

                // move forward/backward for proper distance
                if(tagOfInterest.pose.z > 30)
                {
                    // move forward
                    robot.fLeftWheel.setPower(0.5);
                    robot.fRightWheel.setPower(0.5);
                    robot.bLeftWheel.setPower(0.5);
                    robot.bRightWheel.setPower(0.5);

                    // telemetry
                    telemetry.addData("Moving forward to align with middle tag.", tagOfInterest.pose.z);

                }
                else if (tagOfInterest.pose.z < 30)
                {
                    // move backward
                    robot.fLeftWheel.setPower(-0.5);
                    robot.fRightWheel.setPower(-0.5);
                    robot.bLeftWheel.setPower(-0.5);
                    robot.bRightWheel.setPower(-0.5);

                    // telemetry
                    telemetry.addData("Moving backward to align with middle tag.", tagOfInterest.pose.z);
                }
                else {
                    // stop
                    robot.fLeftWheel.setPower(0);
                    robot.fRightWheel.setPower(0);
                    robot.bLeftWheel.setPower(0);
                    robot.bRightWheel.setPower(0);
                    //score
                    telemetry.addLine("Aligned with middle tag, ready to score.");

                }
            }
            else if(tagOfInterest.id == 1 || tagOfInterest.id == 3)
            {
                // move to the right until the middle tag is in sight
                robot.fLeftWheel.setPower(0.5);
                robot.fRightWheel.setPower(-0.5);
                robot.bLeftWheel.setPower(0.5);
                robot.bRightWheel.setPower(-0.5);

                // telemetry
                telemetry.addData("Moving right to align with middle tag.", tagOfInterest.id);

            }
            else if(tagOfInterest.id == 4 || tagOfInterest.id == 6)
            {
                // move to the left until the middle tag is in sight
                robot.fLeftWheel.setPower(-0.5);
                robot.fRightWheel.setPower(0.5);
                robot.bLeftWheel.setPower(-0.5);
                robot.bRightWheel.setPower(0.5);

                // telemetry
                telemetry.addData("Moving left to align with middle tag.", tagOfInterest.id);
            }
        }
        else
        {
            // move to the right until the middle tag is in sight
            robot.fLeftWheel.setPower(0.5);
            robot.fRightWheel.setPower(-0.5);
            robot.bLeftWheel.setPower(0.5);
            robot.bRightWheel.setPower(-0.5);

            // telemetry
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        }



//        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
//
//            isRight = !isRight;
//        }
//        if (isRight){
//
//            a = Side.Right;
//        }
//        else{
//
//            a = Side.Left;
//        }
//        telemetry.addData("Side",a);
//        telemetry.update();
//
//        if (a == Side.Right) {
//            if(tagOfInterest == null || tagOfInterest.id == Left ) {
//
//
//
//
//
//
//
//
//            } else if(tagOfInterest.id == Middle) {
//                //trajectory
//
//
//            }else {
//                //trajectory
//
//
//            }
//        }
//        else {
//
//
//            /* Actually do something useful */
//            if (tagOfInterest == null || tagOfInterest.id == Left) {
//                //trajectory
//
//
//            } else if (tagOfInterest.id == Middle) {
//
//
//            } else {
//
//            }
//        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
    void tagToArray(AprilTagDetection detection){
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        double[] tagData = new double[2];
        tagData[0] = detection.pose.x*FEET_PER_METER;
        tagData[1] = rot.firstAngle*(180/Math.PI);

    }
}


// Notes
// Translation X is the distance from the camera to the tag in the X direction (left/right)
// Translation Y is the distance from the camera to the tag in the Y direction (up/down)
// Translation Z is the distance from the camera to the tag in the Z direction (forward/backward)
// Rotation Yaw is the rotation of the tag around the Y axis (left/right rotation)
// Rotation Pitch is the rotation of the tag around the X axis (up/down rotation)
// Rotation Roll is the rotation of the tag around the Z axis (tilt left/right rotation)


