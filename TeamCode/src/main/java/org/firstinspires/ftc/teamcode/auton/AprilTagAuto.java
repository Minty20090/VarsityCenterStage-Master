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

@Autonomous(name = "AprilTag Auto")
public class AprilTagAuto extends LinearOpMode {

    private HWMap robot = new HWMap();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // Tag size in meters
    double tagsize = 0.05;

    AprilTagDetection tagOfInterest = null;

    // Constants for movement and alignment
    final double ALIGNMENT_THRESHOLD_X = 0.1; // in meters
    final double OPTIMAL_DISTANCE_Z = 0.1; // in meters
    final double DISTANCE_THRESHOLD = 0.1; // in meters
    final double ALIGNMENT_THRESHOLD_YAW = 5.0;
    final double LEFT_POWER = -0.3;
    final double RIGHT_POWER = 0.3;
    final double FORWARD_POWER = 0.3;
    final double BACKWARD_POWER = -0.3;
    final double ROTATE_POWER = 0.5;

    boolean isBlue = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPSIDE_DOWN);

            }

            @Override
            public void onError(int errorCode) {
                // Handle camera open error
                telemetry.addData("Error", "Camera open failed with error: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            AprilTagDetection bestTag = null;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 1 || tag.id == 4) {
                    if (bestTag == null || Math.abs(tag.pose.x) < Math.abs(bestTag.pose.x)) {
                        bestTag = tag;
                    }
                }
            }

            tagOfInterest = bestTag;

            if (tagOfInterest != null) {

                if (tagOfInterest.pose.z < 0.1) {


                    moveLaterally(isBlue ? LEFT_POWER : RIGHT_POWER);
                    telemetry.addData("Status", "Strafing to find tag");
                    telemetry.addData("Tag ID", tagOfInterest.id);
                    telemetry.addData("X Distance", tagOfInterest.pose.x);
                    telemetry.addData("Z Distance", tagOfInterest.pose.z);
                    telemetry.update();
                } else {
                    stopMovement();
                }


                if (tagOfInterest.pose.z > 0.1) {
                    moveForward();
                    telemetry.addData("Status", "Moving into scoring position");
                    telemetry.addData("Tag ID", tagOfInterest.id);
                    telemetry.addData("X Distance", tagOfInterest.pose.x);
                    telemetry.addData("Z Distance", tagOfInterest.pose.z);
                    telemetry.update();
                } else {
                    stopMovement();
                }

            } else {

                moveLaterally(isBlue ? LEFT_POWER : RIGHT_POWER);
                telemetry.addData("Status", "Searching for tag");
                telemetry.update();

            }


        }


    }

    // Utility methods for robot movement
    void moveLaterally(double power) {
        robot.fLeftWheel.setPower(power);
        robot.fRightWheel.setPower(-power);
        robot.bLeftWheel.setPower(-power);
        robot.bRightWheel.setPower(power);

        //telemetry
        telemetry.addData("Status", "Moving Laterally");
    }

    void moveForward() {
        robot.fLeftWheel.setPower(FORWARD_POWER);
        robot.bLeftWheel.setPower(FORWARD_POWER);
        robot.fRightWheel.setPower(FORWARD_POWER);
        robot.bRightWheel.setPower(FORWARD_POWER);

        //telemetry
        telemetry.addData("Status", "Moving Forward");
    }

    void moveBackward() {
        robot.fLeftWheel.setPower(BACKWARD_POWER);
        robot.bLeftWheel.setPower(BACKWARD_POWER);
        robot.fRightWheel.setPower(BACKWARD_POWER);
        robot.bRightWheel.setPower(BACKWARD_POWER);

        //telemetry
        telemetry.addData("Status", "Moving Backward");
    }

    void stopMovement() {
        robot.fLeftWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        robot.fRightWheel.setPower(0);
        robot.bRightWheel.setPower(0);
    }
}
