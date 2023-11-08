package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "BasicTestTeleop")
public class EncoderCountTests extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();


    @Override
    public void runOpMode() throws InterruptedException {
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

            if (gamepad1.a) {
                tiles(1);
            }
            if (gamepad1.b) {
                turn(90, -1);
            }
            if (gamepad1.x) {
                turn(90, 1);
            }
            if (gamepad1.y) {
                tiles(1);
                turn(90, -1);
            }
        }


    }
    public void tiles(double tiles){
        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();

        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * -600));
        robot.fRightWheel.setTargetPosition((int)(fright + tiles * -600));
        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * -600));
        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * -600));
        robot.fLeftWheel.setPower(.8);
        robot.fRightWheel.setPower(.8);
        robot.bLeftWheel.setPower(.8);
        robot.bRightWheel.setPower(.8);
    }


    public void turn(int degrees, double direction){
        String turn;

        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();
        if (direction < 0) {
            turn = "right";
        }
        else {
            turn = "left";
        }

        if (turn == "left") {
            robot.fLeftWheel.setTargetPosition((fleft));
            robot.fRightWheel.setTargetPosition((int) (fright + (degrees/90 * 564)));
            robot.bLeftWheel.setTargetPosition((int) (bleft + (degrees/90 * -351)));
            robot.bRightWheel.setTargetPosition((int) (bright+ degrees/90 * 201));
        }
        if (turn == "right") {
            robot.fLeftWheel.setTargetPosition((int) (fleft + (degrees/90 * -11)));
            robot.fRightWheel.setTargetPosition((int) (fright + (degrees/90 * -562)));
            robot.bLeftWheel.setTargetPosition((int) (bleft + (degrees/90 * 300)));
            robot.bRightWheel.setTargetPosition((int) (bright+ (degrees/90 * -233)));
        }

        robot.fLeftWheel.setPower(.8);
        robot.fRightWheel.setPower(.8);
        robot.bLeftWheel.setPower(.8);
        robot.bRightWheel.setPower(.8);
    }

}



