package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Projects.HWMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;
import org.firstinspires.ftc.teamcode.auton.BluePropDetectionPipeline.BluePropLocation;
import org.firstinspires.ftc.teamcode.auton.RedPropDetectionPipeline.RedPropLocation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class gyroauto extends LinearOpMode{
    public HWMap robot = new HWMap();
    int noU = 1000;
    OpenCvCamera webcam;
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
//        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRightWheel.setTargetPosition(0);
        robot.fLeftWheel.setTargetPosition(0);
        robot.bRightWheel.setTargetPosition(0);
        robot.bLeftWheel.setTargetPosition(0);
        robot.lift.setTargetPosition(0);
        robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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

                //robot.lift.setPower(.5);
                //  sleep(1000);
                // robot.lift.setPower(0);
                robot.clawR.setPosition(1);
                robot.clawL.setPosition(0);
                robot.lift.setPower(.5);
                robot.lift.setTargetPosition(100);
                sleep(1000);
                robot.lift.setTargetPosition(0);
                sleep(20);

                // START COMMETNED OUT SECTION
              //  sleep(20);
                if(side==1) {
                    //Blue stage

                    spikeB(location);
//                    sleep(500);
//                    tiles(-.9);
//                    sleep(500);
//                    robot.lift.setTargetPosition(0);
//                    sleep(500);
//                    turn(-90);
//                    sleep(500);
//                    tiles(2);
                    break;
                }
                if(side==2){
                    //Blue back stage
                    spikeB(location);
//                    sleep(500);
//                    tiles(-.9);
//                    sleep(500);
//                    robot.lift.setTargetPosition(0);
//                    sleep(500);
//                    turn(-90);
//                    sleep(500);
//                    tiles(3.5);
                    break;

                }
                if(side == 3){
                    //Red backstage
                    spikeR(location);
//                    sleep(500);
//                    tiles(-.9);
//                    sleep(500);
//                    robot.lift.setTargetPosition(0);
//                    sleep(500);
//                    turn(90);
//                    sleep(500);
//                    tiles(2);
                    break;

                }
                if(side == 4) {
                    spikeR(location);
//                    sleep(500);
//                    tiles(-.9);
//                    sleep(500);
//                    robot.lift.setTargetPosition(0);
//                    sleep(500);
//                    turn(90);
//                    sleep(500);
//                    tiles(3);
                    break;
                }
                break;

// END COMMETNED OUT SECTION
            }


        }
    }
    public void drop(){
        robot.clawR.setPosition(.4);
        robot.lift.setTargetPosition(200);
        robot.clawR.setPosition(1);
    }
    public void spikeB(String location) { // blue
        if (location == "Middle") {
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
            System.out.println("bet");
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

//
        }
    }
    public void tiles(double tiles){
        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();

        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * -480));
        robot.fRightWheel.setTargetPosition((int)(fright + tiles * -480));
        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * -600));
        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * -600));
        robot.fLeftWheel.setPower(.5);
        robot.fRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
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

//    public void turnRight(double degrees){
//
//        resetAngle();
//
//        double error = degrees;
//
//        while(opModeIsActive()&&Math.abs(error)>2){
//            double motorPower = (error < 0?-0.3:0.3);
//            setMotorPower(-motorPower, motorPower,-motorPower, motorPower);
//            error = degrees - getAngle();
//            telemetry.addData("error", error);
//            telemetry.update();
//        }
//        robot.fRightWheel.setPower(0);
//        robot.fLeftWheel.setPower(0);
//        robot.bRightWheel.setPower(0);
//        robot.bLeftWheel.setPower(0);
//    }
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
        if(frmotorPower!=0) {
            robot.fRightWheel.setTargetPosition(robot.fRightWheel.getCurrentPosition() + (int)(frmotorPower*4)*8);
        }
        if(frmotorPower!=0) {
            robot.fLeftWheel.setTargetPosition(robot.fLeftWheel.getCurrentPosition() + (int)(flmotorPower*4)*8);
        }
        if(frmotorPower!=0) {
            robot.bRightWheel.setTargetPosition(robot.bRightWheel.getCurrentPosition() + (int)(brmotorPower*4)*8);
        }
        if(frmotorPower!=0) {
            robot.bLeftWheel.setTargetPosition(robot.bLeftWheel.getCurrentPosition() + (int)(blmotorPower*4)*8);
        }

    }
    public void setALLPower(double power) {
        robot.fRightWheel.setPower(power);
        robot.fLeftWheel.setPower(power);
        robot.bRightWheel.setPower(power);
        robot.bLeftWheel.setPower(power);
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
    int WaitTillTargetReached(int tolerance,boolean lock){
        int leftDifference = Math.abs(robot.lift.getTargetPosition() - robot.lift.getCurrentPosition());
        // int rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());

        while(leftDifference > tolerance )

        {
            leftDifference = Math.abs(robot.lift.getTargetPosition() - robot.lift.getCurrentPosition());
            //rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());

            robot.lift.setPower(0.5);
            //robot.rightLift.setPower(0.5);
            sleep(1);
        }
        int a = robot.lift.getCurrentPosition();
        // int c = robot.rightLift.getCurrentPosition();
        int position = (a );


        if(!lock)
        {
            robot.lift.setPower(0);

        }
        return(position);
    }
    private void cycle() {

    }

}