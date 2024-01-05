package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Projects.FleaFlickerMap;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.auton.BasicAuto;
import org.firstinspires.ftc.teamcode.auton.RedPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.auton.BluePropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.auton.BluePropDetectionPipeline.BluePropLocation;
import org.firstinspires.ftc.teamcode.auton.RedPropDetectionPipeline.RedPropLocation;
import java.util.ArrayList;

@Autonomous
public class gyro extends LinearOpMode{
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
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    public boolean wtfTeammate = false;
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
                sleep(500);
                break;
            case 2:
                telemetry.addLine("lBlue");
                telemetry.update();
                sleep(500);
                break;
            case 3:
                telemetry.addLine("rRed");
                telemetry.update();
                sleep(500);
                break;
            case 4:
                telemetry.addLine("lRed");
                telemetry.update();
                sleep(500);
                break;
        }
        if(gamepad1.left_bumper==true&&wtfTeammate==false){
            wtfTeammate = true;
            sleep(500);
        }
        else if(gamepad1.left_bumper==true&&wtfTeammate==true){
            wtfTeammate=false;
            sleep(500);
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
                robot.lift.setTargetPosition(200);
                sleep(1000);
                robot.lift.setTargetPosition(0);
                sleep(20);
                if(side==1) {
                    //Blue backstage
                    spikeB(location);
                    sleep(500);
                    tiles(-.9);
                    sleep(500);
                    robot.lift.setTargetPosition(0);
                    sleep(500);
                    turn(-90);
                    sleep(500);
                    tiles(2);
                }
                if(side==2){
                    //Blue Stage
                    spikeB(location);
                    sleep(500);
                    tiles(-.9);
                    sleep(500);
                    robot.lift.setTargetPosition(0);
                    sleep(500);
                    turn(-90);
                    sleep(500);
                    tiles(3.5);

                }
                if(side==3){
                    //Red backstage
                    spikeR(location);
                    sleep(500);
                    tiles(-.9);
                    sleep(500);
                    robot.lift.setTargetPosition(0);
                    sleep(500);
                    turn(90);
                    sleep(500);
                    tiles(2);
                }
                else {
                    //Red stage - Far
                    spikeR(location);
                    sleep(500);
                    tiles(-.9);
                    sleep(500);
                    robot.lift.setTargetPosition(0);
                    sleep(500);
                    turn(90);
                    sleep(500);
                    tiles(3);
                }
                if(wtfTeammate == true){
                    //different path
                }
                else{
                    //same path
                }


            }


        }
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
        robot.clawR.setPosition(.4);
        robot.lift.setTargetPosition(200);
        robot.clawR.setPosition(1);

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
            turn(90);
            sleep(500);
            drop();
            sleep(2000);
            turn(-90);

        }
        else if(location == "Left"){
            tiles(1);
            turn(-90);
            sleep(500);
            drop();
            sleep(2000);
            turn(90);



        }
    }
    public void spikeR(String location) {
        if (location == "Middle") {
            tiles(1);
            sleep(500);
            drop();




        }
        else if(location == "Right"){
            tiles(1);
            turn(90);
            sleep(500);
            drop();
            turn(-90);


        }
        else if(location == "Left"){
            tiles(1);
            turn(-90);
            sleep(500);
            drop();
            turn(90);


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
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if(deltaAngle > 180){
            deltaAngle -= 360;
        }
        else if(deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;

    }

    public void turnRight(double degrees){

        resetAngle();

        double error = degrees;

        while(opModeIsActive()&&Math.abs(error)>2){
            double motorPower = (error < 0?-0.3:0.3);
            setMotorPower(-motorPower, motorPower,-motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
    }
    //
    public void turn(double degrees){

        resetAngle();

        double error = degrees;

        while(opModeIsActive()&&Math.abs(error)>2){
            double motorPower = (error < 0?-0.3:0.3);
            setMotorPower(-motorPower, motorPower,-motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
    }
    //

    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double error = degrees - orientation.firstAngle;

        if(error>180){
            error -=360;
        }
        else if(error<-180){
            error+=360;
        }
        //
//        if (error > 0) {
//            turnRight(error);
//        }
//        if (error < 0) {
//            turnLeft(error);
//        }
        turn(error);
//        if(degrees>=0){
//            turnRight(degrees);
//        }
//        else{
//            turnLeft(degrees);
//        }
        //

    }
    public void setMotorPower(double frmotorPower, double flmotorPower,double brmotorPower, double blmotorPower) {
        robot.fRightWheel.setPower(frmotorPower);
        robot.fRightWheel.setTargetPosition(robot.fRightWheel.getCurrentPosition()+10);
        robot.fLeftWheel.setPower(flmotorPower);
        robot.fLeftWheel.setTargetPosition(robot.fLeftWheel.getCurrentPosition()+10);
        robot.bRightWheel.setPower(brmotorPower);
        robot.bRightWheel.setTargetPosition(robot.bRightWheel.getCurrentPosition()+10);
        robot.bLeftWheel.setPower(blmotorPower);
        robot.bLeftWheel.setTargetPosition(robot.bLeftWheel.getCurrentPosition()+10);
    }
    public void setALLPower(double power) {
        robot.fRightWheel.setPower(power);
        robot.fLeftWheel.setPower(power);
        robot.bRightWheel.setPower(power);
        robot.bLeftWheel.setPower(power);
    }

}