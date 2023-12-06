package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Projects.HWMap;

import org.firstinspires.ftc.teamcode.Projects.FleaFlickerMap;

@TeleOp(name = "ExtremitiesTest")
public class FlickerTest extends LinearOpMode {
    public HWMap robot = new HWMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.lift.setTargetPosition(0);
        robot.flip.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double speed = .9;

        waitForStart();
        boolean isSpinning = false;
        boolean gateOpen = false;
        boolean clawsOpen = false;
        int intakePosition = 0;
        int outtakePosition = 0;

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

//             Controls
//             a/b - open/close outtake gate
//             x/y - open/close claws
//             right/left trigger - move intake arm up/down
//             right/left bumper - move lift up/down

            if(gamepad1.a && !gateOpen){
                robot.clawR.setPosition(1);
                gateOpen = true;
            }
            else if(gamepad1.b && gateOpen){
                robot.clawR.setPosition(0);
                gateOpen = false;
            }

            if(gamepad1.x && !clawsOpen){
                robot.clawL.setPosition(1);
                //robot.clawR.setPosition(1);
                clawsOpen = true;
            }
            else if(gamepad1.y && clawsOpen){
                robot.clawL.setPosition(0);
                //robot.clawR.setPosition(0);
                clawsOpen = false;
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

            if(gamepad1.right_bumper){
                outtakePosition += 10;
                robot.flip.setPower(.5);
                robot.flip.setTargetPosition(intakePosition);
                telemetry.addData("outtake encoder: ",robot.flip.getCurrentPosition() );
                telemetry.update();
            }
            else if(gamepad1.left_bumper){

                robot.flip.setPower(.5);
                outtakePosition -= 10;
                robot.flip.setTargetPosition(intakePosition);
                telemetry.addData("outtake encoder: ",robot.flip.getCurrentPosition() );
                telemetry.update();
            }



        }

    }

}



