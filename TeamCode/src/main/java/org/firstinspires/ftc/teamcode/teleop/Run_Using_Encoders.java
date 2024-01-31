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
        waitForStart();

        robot.wrist.setPosition(1);
        while (opModeIsActive()) {

            if (gamepad1.a) {
                tiles(1);
            }
            if (gamepad1.b) {
                backTiles(1);
            }
            if (gamepad1.x) {
                correctionLeft(1);
            }
            if (gamepad1.a) {
                correctionRight(1);
            }
        }

    }

    int power = 500;
    public void tiles(double tiles){
        robot.fLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts fl", robot.fLeftWheel.getCurrentPosition());
        robot.fRightWheel.setVelocity(power);
        telemetry.addData("encoder counts fr", robot.fRightWheel.getCurrentPosition());
        robot.bLeftWheel.setVelocity(power);
        telemetry.addData("encoder counts bl", robot.bLeftWheel.getCurrentPosition());
        robot.bRightWheel.setVelocity(power);
        telemetry.addData("encoder counts br", robot.bRightWheel.getCurrentPosition());
        sleep((int) (800*tiles));
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);

    }
    public void backTiles(double tiles) {
        robot.fLeftWheel.setVelocity(-power);
        robot.fRightWheel.setVelocity(-power);
        robot.bLeftWheel.setVelocity(-power);
        robot.bRightWheel.setVelocity(-power);
        sleep((int) (800*tiles));
        robot.fLeftWheel.setVelocity(0);
        robot.fRightWheel.setVelocity(0);
        robot.bLeftWheel.setVelocity(0);
        robot.bRightWheel.setVelocity(0);
    }


    public void correctionLeft( double tiles) {
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



