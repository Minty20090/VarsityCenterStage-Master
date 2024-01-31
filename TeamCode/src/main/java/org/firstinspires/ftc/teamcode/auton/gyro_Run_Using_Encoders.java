package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.teamcode.Projects.HWMapDCex;
import org.firstinspires.ftc.teamcode.auton.BluePropDetectionPipeline.BluePropLocation;
import org.firstinspires.ftc.teamcode.auton.RedPropDetectionPipeline.RedPropLocation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class gyro_Run_Using_Encoders extends LinearOpMode{
    public HWMapDCex robot = new HWMapDCex();
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
            //tiles(1);
            sleep(10000);

                robot.clawR.setPosition(1);
                robot.clawL.setPosition(0);
                sleep(500);
                robot.lift.setPower(.5);
                robot.lift.setTargetPosition(70);
                sleep(1000);
                robot.lift.setTargetPosition(-20);
                sleep(500);
                robot.lift.setTargetPosition(0);
                sleep(100);


                if(side==1) {  // rblue
                    //Blue stage
                    spikeRight(location);
//                    sleep(500);
//                    backTiles(-.9);
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
                    spikeLeft(location);
//                    sleep(500);
//                    backTiles(-.9);
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
                    spikeRight(location);
//                    sleep(500);
//                    backTiles(-.9);
//                    sleep(500);
//                    robot.lift.setTargetPosition(0);
//                    sleep(500);
//                    turn(90);
//                    sleep(500);
//                    tiles(2);
                    break;

                }
                if(side == 4) {
                    // red stage //lred
                    spikeLeft(location);
//                    sleep(500);
//                    backTiles(-.9);
//                    sleep(500);
//                    robot.lift.setTargetPosition(0);
//                    sleep(500);
//                    turn(90);
//                    sleep(500);
//                    tiles(3);
                    break;
                }
            }


        }
    }
    public void drop(){
        robot.clawR.setPosition(0);
        sleep(1000);
        //backTiles(.2);
//        sleep(500);
//        robot.lift.setTargetPosition(100);
//        sleep(500);
////        tiles(.2);
//        robot.clawR.setPosition(1);
//        sleep(500);
    }
    public void spikeLeft(String location) { // tress is to the right
        if (location == "Middle") {
           // tiles(.9);
            sleep(500);
            drop();
            robot.clawR.setPosition(0);
            sleep(1000);
//            sleep(2000);

        }
        else if(location == "Right"){
           // tiles(.8);
            turn(-85);
            sleep(500);
          //  correctionLeft(.4);
            sleep(500);
            sleep(500);
            drop();
            robot.clawR.setPosition(0);
            sleep(1000);
//            turn(85);
        }
        else if(location == "Left"){
           // tiles(1);
            turn(85);

            sleep(500);
           // backTiles(.3);
            drop();
            robot.clawR.setPosition(0);
            sleep(1000);
//            sleep(2000);
//            turn(-85);

        }
    }
    public void spikeRight(String location) {// tress is to the left
        if (location == "Middle") {
            //tiles(.9);
            sleep(500);
            drop();
            robot.clawR.setPosition(0);
            sleep(1000);
        }
        else if(location == "Right"){
           // tiles(1);
            turn(-85);
            sleep(500);
            //backTiles(.1);
            sleep(500);
            drop();
            robot.clawR.setPosition(0);
            sleep(1000);
//            turn(85);

        }
        else if(location == "Left"){
           // tiles(.8);
            turn(85);
            sleep(500);
           // backTiles(.2);
            drop();
            robot.clawR.setPosition(0);
            sleep(1000);
//            turn(-85);

//
        }
    }
    int power = 1000;
    public void tiles(double tiles){
        robot.fLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts fl", robot.fLeftWheel.getCurrentPosition());
        robot.fRightWheel.setVelocity(power);
        telemetry.addData("encoder counts fr", robot.fRightWheel.getCurrentPosition());
        robot.bLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts bl", robot.bLeftWheel.getCurrentPosition());
        robot.bRightWheel.setVelocity(power);
        telemetry.addData("encoder counts br", robot.bRightWheel.getCurrentPosition());
        sleep((int) (800*tiles));
        telemetry.update();
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);
    }
    public void backTiles(double tiles) {
        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(-power);
        sleep((int) (800*tiles));
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);
    }


    public void correctionLeft( double tiles) {
        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(power);
        robot.bLeftWheel.setVelocity(power);
        robot.bRightWheel.setVelocity(-power);
        sleep((int) (800*tiles));
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);
    }
    public void correctionRight( double tiles) {

        robot.fLeftWheel.setVelocity(power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(power);
        sleep((int) (800*tiles));
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);
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
//        robot.fLeftWheel.setVelocity(0);
//        robot.fRightWheel.setVelocity(0);
//        robot.bLeftWheel.setVelocity(0);
//        robot.bRightWheel.setVelocity(0);
    }
    //

    public void turnTo(double degrees) {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        turn(error);

    }
    public void setALLPower(double power) {
//        robot.fRightWheel.setVelocity(power);
//        robot.fLeftWheel.setVelocity(-power);
//        robot.bRightWheel.setVelocity(power);
//        robot.bLeftWheel.setVelocity(-power);
    }




}