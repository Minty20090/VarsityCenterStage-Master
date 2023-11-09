package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "EncoderCountTests")
public class EncoderCountTests extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.fRightWheel.setTargetPosition(0);
        robot.fLeftWheel.setTargetPosition(0);
        robot.bRightWheel.setTargetPosition(0);
        robot.bLeftWheel.setTargetPosition(0);
        robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("Counts:", "BL=%d FL=%d BR=%d FR=%d", robot.bLeftWheel.getCurrentPosition(), robot.fLeftWheel.getCurrentPosition(), robot.bRightWheel.getCurrentPosition(), robot.fRightWheel.getCurrentPosition());
            telemetry.addData("Motor Power:", String.valueOf(robot.bLeftWheel.getPower()), String.valueOf(robot.fLeftWheel.getPower()), String.valueOf(robot.bRightWheel.getPower()), String.valueOf(robot.fRightWheel.getPower()));
            telemetry.addData("Target Counts:", "BL=%d FL=%d BR=%d FR=%d", robot.bLeftWheel.getTargetPosition(), robot.fLeftWheel.getTargetPosition(), robot.bRightWheel.getTargetPosition(), robot.fRightWheel.getTargetPosition());
            telemetry.addData("IsBusy:", String.valueOf(robot.bLeftWheel.isBusy()), robot.fLeftWheel.isBusy(), robot.bRightWheel.isBusy(), robot.fRightWheel.isBusy());

            if (robot.bLeftWheel.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                telemetry.addData("right mode", true);
            }
            telemetry.update();
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
        robot.fLeftWheel.setPower(.5);
        robot.fRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * -600));
        robot.fRightWheel.setTargetPosition((int)(fright + tiles * -600));
        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * -600));
        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * -600));

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
        sleep(3000);
    }

}



