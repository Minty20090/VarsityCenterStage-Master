package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Projects.HWMap;

@TeleOp(name = "manualtriglift")
public class manualtriglift extends LinearOpMode {
    public HWMap robot = new HWMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.ext.setTargetPosition(0);
        robot.ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double speed = .7;

        waitForStart();
        boolean isSpinning = false;
        int liftPosition = 0;
        int jointPosition = 0;
        int noU = 1000;
        boolean gateOpen = false;
        boolean clawsOpen = false;

        while (opModeIsActive()) {
//            OpenCvCamera webcam;
//            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
//
//
//            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//
//                }
//            });



            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(frontLeftPower*speed);
            robot.bLeftWheel.setPower(backLeftPower*speed);
            robot.fRightWheel.setPower(frontRightPower*speed);
            robot.bRightWheel.setPower(backRightPower*speed);

            // Teleop Code goes here

            if(gamepad1.right_trigger > 0){
                robot.clawR.setPosition(1);
            }
            if(gamepad1.right_bumper){
                robot.clawR.setPosition(.8);
            }
            if(gamepad1.left_trigger > 0){
                robot.clawL.setPosition(.4);
            }
            if(gamepad1.left_bumper){
                robot.clawL.setPosition(.7);

            }

            if(gamepad1.dpad_up){
                jointPosition = robot.ext.getCurrentPosition() + 50;
                robot.ext.setPower(.5);
                robot.ext.setTargetPosition(jointPosition);

            }
            else if(gamepad1.dpad_down){
                robot.ext.setPower(.5);
                jointPosition = robot.ext.getCurrentPosition() - 50;
                robot.ext.setTargetPosition(jointPosition);

            }
            if(gamepad1.a&&robot.lift.getCurrentPosition()>=180){
                robot.lift.setPower(-.3*(Math.sin(robot.lift.getCurrentPosition()/2*(Math.PI/180))));
            }
            else if(gamepad1.a&&robot.lift.getCurrentPosition()<180){
                robot.lift.setPower(-.05*(Math.sin(robot.lift.getCurrentPosition()/2*(Math.PI/180))));
            }
            else if(gamepad1.y&&robot.lift.getCurrentPosition()<=180){
                robot.lift.setPower(.3*(Math.cos(robot.lift.getCurrentPosition()/2*(Math.PI/180))));
            }
            else if(gamepad1.y&&robot.lift.getCurrentPosition()>180){
                robot.lift.setPower(.05*(Math.cos(robot.lift.getCurrentPosition()/2*(Math.PI/180))));
            }
            else {
                robot.lift.setPower(0);
            }

            telemetry.addData("lift: %d", robot.lift.getCurrentPosition());
            telemetry.addData("power: %d",robot.lift.getPower());
            telemetry.update();


        }

    }
}



