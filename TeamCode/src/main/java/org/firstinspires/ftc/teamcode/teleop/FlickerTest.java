package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Projects.FleaFlickerMap;

@TeleOp(name = "ExtremitiesTest")
public class FlickerTest extends LinearOpMode {
    public FleaFlickerMap robot = new FleaFlickerMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double speed = .9;

        waitForStart();
        boolean isSpinning = false;
        boolean gateOpen = false;
        boolean clawsOpen = false;
        double intakePosition = 0;

        while (opModeIsActive()) {

            // Teleop Code goes here
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


            if(gamepad1.a && !gateOpen){
                robot.gate.setPosition(1);
                gateOpen = true;
            }
            else if(gamepad1.a && gateOpen){
                robot.gate.setPosition(0);
                gateOpen = false;
            }

            if(gamepad1.b && !clawsOpen){
                robot.clawL.setPosition(1);
                robot.clawR.setPosition(0);
                clawsOpen = true;
            }
            else if(gamepad1.b && clawsOpen){
                robot.clawL.setPosition(0);
                robot.clawR.setPosition(1);
                clawsOpen = false;
            }

            if(gamepad1.right_trigger > 0){
                intakePosition += 0.1;
            }
            else if(gamepad1.left_trigger > 0){
                intakePosition -= 0.1;
            }

            robot.intakeR.setPosition(intakePosition);
            robot.intakeL.setPosition(intakePosition);

            if(gamepad1.right_bumper) {
                robot.lift.setPower(1);
            } else if (gamepad1.left_bumper){
                robot.lift.setPower(-1);
            } else{
                robot.lift.setPower(0);
            }

        }

    }
    public void showEncoderCounts(int fleft, int fright, int bleft, int bright, int slide){
        telemetry.addData("F Left: ", Integer.toString(fleft));
        telemetry.addData("F Right: ", Integer.toString(fright));
        telemetry.addData("B Left: ", Integer.toString(bleft));
        telemetry.addData("B Right: ", Integer.toString(bright));
        telemetry.addData("slide: ", Integer.toString(slide));
    }


}



