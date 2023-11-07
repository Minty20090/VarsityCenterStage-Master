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
        robot.init(hardwareMap);


        double speed = .9;

        waitForStart();
        boolean isSpinning = false;
        int intakePosition = 0;
        boolean gateOpen = false;
        boolean clawsOpen = false;

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y; // Remember, this is reversed!
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

            // show wheel encoder values in telemetry
            telemetry.addData("fLeftWheel", robot.fLeftWheel.getCurrentPosition());
            telemetry.addData("fRightWheel", robot.fRightWheel.getCurrentPosition());
            telemetry.addData("bLeftWheel", robot.bLeftWheel.getCurrentPosition());
            telemetry.addData("bRightWheel", robot.bRightWheel.getCurrentPosition());



            // ============================================
            //                     ICI
            // ============================================
            if (gamepad1.a) {
                robot.stick.setPosition(0);
            } else if (gamepad1.b) {
                robot.stick.setPosition(1);
            }
            
            if (gamepad1.x) {
                double currentPosition = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition + 5.00);
            } else if (gamepad1.y) {
                double currentPosition = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition - 5.00);
            }
            if(gamepad1.right_trigger > 0){
                intakePosition += 10;
                robot.lift.setPower(.5);
                robot.lift.setTargetPosition(intakePosition);
            }
            else if(gamepad1.left_trigger > 0){
                robot.lift.setPower(.5);
                intakePosition -= 10;
                robot.lift.setTargetPosition(intakePosition);
            }
            if(gamepad1.a && !gateOpen){
                robot.wrist.setPosition(1);
                gateOpen = true;
            }
            else if(gamepad1.b && gateOpen){
                robot.wrist.setPosition(0);
                gateOpen = false;
            }

            if(gamepad1.x && !clawsOpen){
                robot.stick.setPosition(1);

                clawsOpen = true;
            }
            else if(gamepad1.y && clawsOpen){
                robot.stick.setPosition(0);

                clawsOpen = false;
            }



        }

    }

}



