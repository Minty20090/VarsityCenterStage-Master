package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "encoderTest")
public class encoderTest extends LinearOpMode {
    public HWMap robot = new HWMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        double speed = .7;

        waitForStart();
        boolean isSpinning = false;
        robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y ; // Remember, this is reversed!
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

            telemetry.addData("Counts:", "BL=%d FL=%d BR=%d FR=%d Lift=%d", robot.bLeftWheel.getCurrentPosition(), robot.fLeftWheel.getCurrentPosition(), robot.bRightWheel.getCurrentPosition(), robot.fRightWheel.getCurrentPosition(), robot.lift.getCurrentPosition()   );

            telemetry.update();

            if (gamepad1.a) {
                robot.fLeftWheel.setPower(.5);
                robot.bLeftWheel.setPower(.5);
                robot.fRightWheel.setPower(.5);
                robot.bRightWheel.setPower(.5);
            }
            else if (gamepad1.y) {
                robot.fLeftWheel.setPower(-.5);
                robot.bLeftWheel.setPower(-.5);
                robot.fRightWheel.setPower(-.5);
                robot.bRightWheel.setPower(-.5);
            }
            else if (gamepad1.b){
                robot.fLeftWheel.setPower(-.5);
                robot.bLeftWheel.setPower(-.5);
                robot.fRightWheel.setPower(.5);
                robot.bRightWheel.setPower(.5);
            }
            else if (gamepad1.x){
                robot.fLeftWheel.setPower(.5);
                robot.bLeftWheel.setPower(.5);
                robot.fRightWheel.setPower(-.5);
                robot.bRightWheel.setPower(-.5);
            }
            else{
                robot.fLeftWheel.setPower(0);
                robot.bLeftWheel.setPower(0);
                robot.fRightWheel.setPower(0);
                robot.bRightWheel.setPower(0);
            }

        }

    }

}


