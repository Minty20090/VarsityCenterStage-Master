package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Projects.HWMap;

@TeleOp(name = "encoderTest")
public class encoderTest extends LinearOpMode {
    public HWMap robot = new HWMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.fLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double speed = .9;

        waitForStart();
        boolean isSpinning = false;

        while (opModeIsActive()) {

            // Teleop Code goes here

            if (gamepad1.a) {

            }
            if (gamepad1.b) {
                robot.bRightWheel.setTargetPosition();
            }

            if (gamepad1.x) {
                double currentPosition = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition + 5.00);
            } else if (gamepad1.y) {
                double currentPosition = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition - 5.00);
            }



        }

    }

}



