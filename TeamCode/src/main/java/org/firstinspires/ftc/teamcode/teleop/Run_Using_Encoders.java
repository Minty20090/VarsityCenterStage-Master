package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.Projects.HWMapDCex;

@TeleOp(name = "Run_Using_Encoders")
public class Run_Using_Encoders extends LinearOpMode {
    public HWMapDCex robot = new HWMapDCex();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = 0.7;
        waitForStart();


        while (opModeIsActive()) {

            robot.wrist.setPosition(1);

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

            if (gamepad1.a) {
                tiles(1);
            }
            if (gamepad1.b) {
                backTiles(1);
            }
            if (gamepad1.x) {
                correctionLeft(.5);
            }
            if (gamepad1.y) {
                correctionRight(.5);
            }
        }

    }


    public void tiles(double tiles){
        int power = 400;
        robot.fLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts fl", robot.fLeftWheel.getCurrentPosition());
        robot.fRightWheel.setVelocity(power);
        telemetry.addData("encoder counts fr", robot.fRightWheel.getCurrentPosition());
        robot.bLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts bl", robot.bLeftWheel.getCurrentPosition());
        robot.bRightWheel.setVelocity(power);
        telemetry.addData("encoder counts br", robot.bRightWheel.getCurrentPosition());
        sleep((int) (1600*tiles));
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);

    }
    public void backTiles(double tiles) {
        int power = 500;
        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(-power);
        sleep((int) (800*tiles));
        robot.fLeftWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);
    }


    public void correctionLeft( double tiles) {
        int power = 500;
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
        int power = 500;
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

}



