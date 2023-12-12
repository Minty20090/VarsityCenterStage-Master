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
    private ElapsedTime runtime = new ElapsedTime();
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



        while (opModeIsActive()) {

            // NEGATIVE IS LEFT
            // POSITIVE IS RIGHT
            if(gamepad1.a==true){
                turnTo(-90);
            }
            if (gamepad1.b) {
                turnTo(90);
            }
            if(gamepad1.y){
                liftPosition = robot.lift.getTargetPosition() + 10;
                robot.lift.setPower(.5);
                robot.lift.setTargetPosition(liftPosition);

            }
            else if(gamepad1.a){
                robot.lift.setPower(.5);
                liftPosition = robot.lift.getTargetPosition() - 10;
                robot.lift.setTargetPosition(liftPosition);

            }

            if(gamepad1.dpad_up){
                jointPosition = robot.ext.getCurrentPosition() + 10;
                robot.ext.setPower(.5);
                robot.ext.setTargetPosition(jointPosition);

            }
            else if(gamepad1.dpad_down){
                robot.ext.setPower(.5);
                jointPosition = robot.ext.getCurrentPosition() - 10;
                robot.ext.setTargetPosition(jointPosition);

            }
            telemetry.addData("lift:", robot.ext.getCurrentPosition());
            telemetry.addData("joint:", robot.lift.getCurrentPosition());
            telemetry.update();

        }

    }
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if(deltaAngle > 180){
            deltaAngle -= 360;
        }
        else if(deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;

    }

    public void turnRight(double degrees){

        resetAngle();

        double error = degrees;

        while(opModeIsActive()&&Math.abs(error)>2){
            double motorPower = (error < 0?-0.3:0.3);
           setMotorPower(-motorPower, motorPower,-motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        setALLPower(0);
    }
    public void turnLeft(double degrees){

        resetAngle();

        double error = degrees;

        while(opModeIsActive()&&Math.abs(error)>2){
            double motorPower = (error < 0?-0.3:0.3);
            setMotorPower(motorPower, -motorPower,motorPower, -motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        setALLPower(0);
    }

    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double error = degrees - orientation.firstAngle;

        if(error>180){
            error -=360;
        }
        else if(error<-180){
            error+=360;
        }
        if (error > 0) {
            turnRight(error);
        }
        if (error < 0) {
            turnLeft(error);
        }

    }
    public void setMotorPower(double frmotorPower, double flmotorPower,double brmotorPower, double blmotorPower) {
        robot.fRightWheel.setPower(frmotorPower);
        robot.fLeftWheel.setPower(flmotorPower);
        robot.bRightWheel.setPower(brmotorPower);
        robot.bLeftWheel.setPower(blmotorPower);
    }
    public void setALLPower(double power) {
        robot.fRightWheel.setPower(power);
        robot.fLeftWheel.setPower(power);
        robot.bRightWheel.setPower(power);
        robot.bLeftWheel.setPower(power);
    }
}
//    public void turnLeft(double turnAngle, double timeoutS) {
//        sleep(500);
//        //double angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double speed=.5;
//        double oldDegreesLeft=turnAngle;
//        double scaledSpeed=speed;
//        double targetHeading=angles.firstAngle+turnAngle;
//        double oldAngle=angles.firstAngle;
//        if(targetHeading<-180) {targetHeading+=360;}
//        if(targetHeading>180){targetHeading-=360;}
//        double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
//        runtime.reset();
//        while(opModeIsActive() &&
//                runtime.seconds() < timeoutS &&
//                degreesLeft>1&&
//                oldDegreesLeft-degreesLeft>=0) { //check to see if we overshot target
//            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
//            if(scaledSpeed>1){scaledSpeed=.1;}
//            robot.bLeftWheel.setPower(scaledSpeed*1.3); //extra power to back wheels
//            robot.bRightWheel.setPower(-1*scaledSpeed*1.3); //due to extra weight
//            robot.fLeftWheel.setPower(scaledSpeed);
//            robot.fRightWheel.setPower(-1*scaledSpeed);
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            oldDegreesLeft=degreesLeft;
//            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
//            if(Math.abs(angles.firstAngle-oldAngle)<1){speed*=1.1;}. //bump up speed to wheels in case robot stalls before reaching target
//            oldAngle=angles.firstAngle;
//        }


