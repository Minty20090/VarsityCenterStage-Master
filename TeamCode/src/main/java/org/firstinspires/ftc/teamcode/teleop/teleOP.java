package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Projects.HWMap;

@TeleOp(name = "teleOP")
public class teleOP extends LinearOpMode {
    public HWMap robot = new HWMap();
    private double currAngle = 0.0;
    private Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ext.setTargetPosition(0);
        robot.ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.init(hardwareMap);
        double speed = .7;


        waitForStart();
        boolean isSpinning = false;
        int liftPosition = 0;
        int noU = 360;
        boolean isFlip = false;
        int jointPosition = 0;
        boolean gateOpen = false;
        boolean clawsOpen = false;
        int rightPosition = 0;
        int leftPosition = 0;
        //int[] positions;
        double pace = 0.9;


        while (opModeIsActive()) {

            //double y = gamepad1.left_stick_y; // Remember, this is reversed!
            //double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            //double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            // double frontLeftPower = (y + x + rx) / denominator;
            //double backLeftPower = (y - x + rx) / denominator;
            // double frontRightPower = (y - x - rx) / denominator;
            // double backRightPower = (y + x - rx) / denominator;

            // robot.fLeftWheel.setPower(frontLeftPower*speed);
            //robot.bLeftWheel.setPower(backLeftPower*speed);
            // robot.fRightWheel.setPower(frontRightPower*speed);
            // robot.bRightWheel.setPower(backRightPower*speed);

            // Teleop Code goes here

            // show wheel encoder values in telemetry
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


            while(robot.lift.isBusy()&&!isStopRequested()){

            }

            // if (gamepad1.a) {
            // robot.stick.setPosition(0);
            //  } else if (gamepad1.b) {
            //robot.stick.setPosition(1);
            // }


            if(gamepad1.right_trigger > 0){
                robot.clawR.setPosition(1);
            }
            if(gamepad1.right_bumper){
                robot.clawR.setPosition(.5); //open
            }
            else if(gamepad1.left_trigger > 0){
                robot.clawL.setPosition(1);
            }
            if(gamepad1.left_bumper){
                robot.clawL.setPosition(.5);// closes
            }
            //if(button == true&&isFlip==false){
            // robot.flip.setPosition(1);}
            //else if(button == true &&isFlip == true){
            // robot.flip.setPosition(0);}


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



                noU = noU - 1;

                //robot.rightLift.setPower(-.4);

                //robot.rightLift.setTargetPosition(noU);
                robot.lift.setTargetPosition(noU);
                robot.lift.setPower(.3);
                sleep(500);
            }
            else if (gamepad1.dpad_up == true&&noU<360) {

                //robot.lift.setPower(-1);
                //robot.lift.setTargetPosition(liftstart);


                noU = noU+1;

                // robot.rightLift.setPower(.4);

                // robot.rightLift.setTargetPosition(noU);
                robot.lift.setTargetPosition(noU);
                robot.lift.setPower(.3);
                sleep(500);


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
            if(gamepad1.start==true){
                omgGoStraight(1);
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
    public void omgGoStraight(double tiles){

        double target = robot.fRightWheel.getCurrentPosition()+tiles*600;
        double delta = target - robot.fRightWheel.getCurrentPosition();


        resetAngle();

        // Loop until the specified time period ends
        while (delta>0) {
            robot.fRightWheel.setPower(.5);
            robot.bRightWheel.setPower(.5);
            robot.fLeftWheel.setPower(.5);
            robot.bLeftWheel.setPower(.5);

            if(getAngle() > 0){
                robot.fRightWheel.setPower(.7);
                robot.bRightWheel.setPower(.7);
                robot.fLeftWheel.setPower(.5);
                robot.bLeftWheel.setPower(.5);
            }
            else if(getAngle() < 0){
                robot.fRightWheel.setPower(.5);
                robot.bRightWheel.setPower(.5);
                robot.fLeftWheel.setPower(.7);
                robot.bLeftWheel.setPower(.7);
            }
            delta = target - robot.fRightWheel.getCurrentPosition();

            // Optional: Add a small delay to prevent the loop from hogging system resources
            sleep(10);
        }

        // Stop the robot after the loop
        robot.fRightWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
    }
    public void inPlace(){
        int n = robot.lift.getCurrentPosition();
        while(robot.lift.getCurrentPosition()-n>5){
            robot.lift.setPower(-.5);
        }
        while(robot.lift.getCurrentPosition()-n<-5){
            robot.lift.setPower(.5);
        }
    }

}



