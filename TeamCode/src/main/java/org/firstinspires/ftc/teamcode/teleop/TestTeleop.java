package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Projects.HWMap;

@TeleOp(name = "TestTeleop")
public class TestTeleop extends LinearOpMode {
    public HWMap robot = new HWMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

            // show wheel encoder values in telemetry
            telemetry.addData("fLeftWheel", robot.fLeftWheel.getCurrentPosition());
            telemetry.addData("fRightWheel", robot.fRightWheel.getCurrentPosition());
            telemetry.addData("bLeftWheel", robot.bLeftWheel.getCurrentPosition());
            telemetry.addData("bRightWheel", robot.bRightWheel.getCurrentPosition());




            if(gamepad1.right_trigger > 0){
                robot.clawR.setPosition(1);
            }
            if(gamepad1.right_bumper){
                robot.clawR.setPosition(.25);
            }
            if(gamepad1.left_trigger > 0){
                robot.clawL.setPosition(1);
            }
            if(gamepad1.left_bumper){
                robot.clawL.setPosition(.25);
            }


//            if(gamepad1.y){
//                liftPosition = robot.lift.getTargetPosition() + 1;
//               robot.lift.setPower(.5);
//               robot.lift.setTargetPosition(liftPosition);
//                //robot.lift.setTargetPosition(20);
//                //robot.lift.setPower(.75);
//
//
//
//            }
////            else{
////                robot.lift.setPower(0);
////            }
//            if(gamepad1.a){
//                robot.lift.setPower(.5);
//                liftPosition = robot.lift.getTargetPosition() - 1;
//                robot.lift.setTargetPosition(liftPosition);
//               // robot.lift.setPower(-.75);
//            }
//            else{
//                robot.lift.setPower(0);
//            }

            if(gamepad1.dpad_left){
                jointPosition = robot.ext.getCurrentPosition() + 10;
                robot.ext.setPower(.5);
                robot.ext.setTargetPosition(jointPosition);

            }
            else if(gamepad1.dpad_right){
                robot.ext.setPower(.5);
                jointPosition = robot.ext.getCurrentPosition() - 10;
                robot.ext.setTargetPosition(jointPosition);

            }
//            if (gamepad1.x) {
//                robot.lift.setPower(.5);
//                robot.lift.setTargetPosition(0);
//                robot.ext.setTargetPosition(0);
//                robot.ext.setPower(.5);
//            }
            if (gamepad1.dpad_down == true&&noU>0) {


                //robot.lift.setPower(1);
                // robot.lift.setTargetPosition(liftend);



                noU = noU - 5;

                //robot.rightLift.setPower(-.4);
                robot.lift.setPower(-.4);
                //robot.rightLift.setTargetPosition(noU);
                robot.lift.setTargetPosition(noU);
            }
            else if (gamepad1.dpad_up == true&&noU<700) {

                //robot.lift.setPower(-1);
                //robot.lift.setTargetPosition(liftstart);


                noU = noU+5;

                // robot.rightLift.setPower(.4);
                robot.lift.setPower(.4);
                // robot.rightLift.setTargetPosition(noU);
                robot.lift.setTargetPosition(noU);



            }

            else {
                // robot.lift.setPower(0);
            }


            if (gamepad1.b == true){


                // robot.rightLift.setTargetPosition(-350);
                robot.lift.setTargetPosition(100);
                noU = WaitTillTargetReached(50, true);



            }
            else if (gamepad1.y == true){

                // robot.rightLift.setTargetPosition(-850);
                robot.lift.setTargetPosition(200);
                noU = WaitTillTargetReached(50, true);

            }

            else if(gamepad1.x == true){

                // robot.rightLift.setTargetPosition(-1600);
                robot.lift.setTargetPosition(300);
                noU = WaitTillTargetReached(50, true);


            }
            else if (gamepad1.a == true){

                //robot.rightLift.setTargetPosition(0);
                robot.lift.setTargetPosition(0);
                noU = WaitTillTargetReached(50, true);


            }




        }

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



