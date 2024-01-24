package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Projects.HWMapDCex;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "lift")
public class lift extends LinearOpMode{
    public HWMapDCex robot = new HWMapDCex();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double speed = 100.0;

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
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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
                robot.clawR.setPosition(0); //open
            }
            if(gamepad1.left_trigger > 0){
                robot.clawL.setPosition(0);
            }
            if(gamepad1.left_bumper){
                robot.clawL.setPosition(1);//open

            }

            if(gamepad1.dpad_up){
               //robot.lift.setVelocity(speed);
            }
            else if(gamepad1.dpad_down){
               // robot.lift.setVelocity(-speed);

            }
            else{
                //robot.lift.setVelocity(0.0);
            }

//            if(gamepad1.a){
//                robot.lift.setPower(2*(Math.cos(robot.lift.getCurrentPosition()/2)));
//            }
//            else if(gamepad1.y){
//                robot.lift.setPower(2*(Math.cos(robot.lift.getCurrentPosition()/2)));
//            }






            telemetry.addData("lift: %d", robot.lift.getCurrentPosition());
            telemetry.update();


        }


    }

}

