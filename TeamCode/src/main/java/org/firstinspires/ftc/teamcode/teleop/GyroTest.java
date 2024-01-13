package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Projects.HWMap;

@TeleOp(name = "GyroTest")
public class GyroTest extends LinearOpMode {
    public HWMap robot = new HWMap();
    //private ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    int liftPosition = 0;
    int jointPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ext.setTargetPosition(0);
        robot.ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        boolean isSpinning = false;
        int liftPosition = 0;
        int jointPosition = 0;
        boolean gateOpen = false;
        boolean clawsOpen = false;

        while (opModeIsActive()) {

            // NEGATIVE IS LEFT
            // POSITIVE IS RIGHT
            if (gamepad1.x == true) {
                turn(90);

            }
            if (gamepad1.b) {
                turn(-90);
            }

        }

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
        robot.fLeftWheel.setPower(-power);
        robot.bRightWheel.setPower(power);
        robot.bLeftWheel.setPower(-power);
    }
}

