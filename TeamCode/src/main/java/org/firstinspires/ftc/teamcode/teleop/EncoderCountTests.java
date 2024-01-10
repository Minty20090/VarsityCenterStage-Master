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
                tiles(.5);

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
        sleep((int)(1000 * tiles));
        robot.fLeftWheel.setPower(0);
        robot.fRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
//        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * -450));
//        robot.fRightWheel.setTargetPosition((int)(fright + tiles * -500));
//        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * -550));
//        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * -600));
        sleep(2000);

    }
    public void correction( double tiles) {
        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();
        robot.fLeftWheel.setPower(.5);
        robot.fRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
        robot.fLeftWheel.setTargetPosition((int) (fleft + tiles * -70));
        robot.fRightWheel.setTargetPosition((int)(fright + tiles * 70));
        robot.bLeftWheel.setTargetPosition((int)(bleft + tiles * 70));
        robot.bRightWheel.setTargetPosition((int)(bright+ tiles * -70));
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
            robot.bLeftWheel.setTargetPosition((int) (bleft + (degrees/90 * 632)));
            robot.fLeftWheel.setTargetPosition((int) (fleft + (degrees/90 * 365)));
            robot.bRightWheel.setTargetPosition((int) (bright+ degrees/90 * -565));
            robot.fRightWheel.setTargetPosition((int) (fright + (degrees/90 * -259)));

        }
        if (turn == "right") {

            robot.bLeftWheel.setTargetPosition((int) (bleft + (degrees/90 * -528)));
            robot.fLeftWheel.setTargetPosition((int) (fleft + (degrees/90 * -329)));
            robot.bRightWheel.setTargetPosition((int) (bright+ (degrees/90 * 567)));
            robot.fRightWheel.setTargetPosition((int) (fright + (degrees/90 * 264)));
        }

        robot.fLeftWheel.setPower(.8);
        robot.fRightWheel.setPower(.8);
        robot.bLeftWheel.setPower(.8);
        robot.bRightWheel.setPower(.8);
        sleep(3000);
    }

}



