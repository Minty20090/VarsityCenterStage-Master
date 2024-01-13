package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "EncoderCountTests")
public class EncoderCountTests extends LinearOpMode {
    public HWMap robot = new HWMap();
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

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
                turn(90);
            }
            if (gamepad1.y) {
                tiles(1);
                turn(90);
            }
        }


    }
    public void tiles(double tiles){
        int fleft = robot.fLeftWheel.getCurrentPosition();
        int bleft = robot.bLeftWheel.getCurrentPosition();
        int bright = robot.bRightWheel.getCurrentPosition();
        int fright = robot.fRightWheel.getCurrentPosition();

        robot.fLeftWheel.setTargetPosition((int) (fleft + (int)(tiles * 600)));
        robot.fRightWheel.setTargetPosition((int)(fright + (int)(tiles * 600)));
        robot.bLeftWheel.setTargetPosition((int)(bleft + (int)(tiles * 600)));
        robot.bRightWheel.setTargetPosition((int)(bright+ (int)(tiles * 600)));
        robot.fLeftWheel.setPower(.5);
        robot.fRightWheel.setPower(.5);
        robot.bLeftWheel.setPower(.5);
        robot.bRightWheel.setPower(.5);
    }
    public void tilesNoEncoders(double tiles){
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
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;

    }
    public void turn(double degrees) {

        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPower(-motorPower, motorPower,-motorPower,motorPower);
            setALLPower(motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.fRightWheel.setPower(0);
        robot.fLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
    }
    //

    public void turnTo(double degrees) {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        turn(error);

    }

    public void setMotorPower(double frmotorPower, double flmotorPower, double brmotorPower, double blmotorPower) {
        if (frmotorPower != 0) {
            robot.fRightWheel.setTargetPosition(robot.fRightWheel.getCurrentPosition() + (int) (frmotorPower * 4) * 8);
        }
        if (frmotorPower != 0) {
            robot.fLeftWheel.setTargetPosition(robot.fLeftWheel.getCurrentPosition() + (int) (flmotorPower * 4) * 8);
        }
        if (frmotorPower != 0) {
            robot.bRightWheel.setTargetPosition(robot.bRightWheel.getCurrentPosition() + (int) (brmotorPower * 4) * 8);
        }
        if (frmotorPower != 0) {
            robot.bLeftWheel.setTargetPosition(robot.bLeftWheel.getCurrentPosition() + (int) (blmotorPower * 4) * 8);
        }

    }

    public void setALLPower(double power) {
        robot.fRightWheel.setPower(power);
        robot.fLeftWheel.setPower(power);
        robot.bRightWheel.setPower(power);
        robot.bLeftWheel.setPower(power);
    }

}



