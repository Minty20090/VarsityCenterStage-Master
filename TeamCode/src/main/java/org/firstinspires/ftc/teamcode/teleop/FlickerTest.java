package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Projects.FleaFlickerMap;

@TeleOp(name = "ExtremitiesTest")
public class FlickerTest extends LinearOpMode {
    public FleaFlickerMap robot = new FleaFlickerMap();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double speed = .9;

        waitForStart();
        boolean isSpinning = false;
        boolean gateOpen = false;
        boolean clawsOpen = false;
        double intakePosition = 0;

        while (opModeIsActive()) {

            // Teleop Code goes here
            if(gamepad1.a && !gateOpen){
                robot.gate.setPosition(1);
                gateOpen = true;
            }
            else if(gamepad1.a && gateOpen){
                robot.gate.setPosition(0);
                gateOpen = false;
            }

            if(gamepad1.b && !clawsOpen){
                robot.clawL.setPosition(1);
                robot.clawR.setPosition(0);
                clawsOpen = true;
            }
            else if(gamepad1.b && clawsOpen){
                robot.clawL.setPosition(0);
                robot.clawR.setPosition(1);
                clawsOpen = false;
            }

            if(gamepad1.right_trigger > 0){
                intakePosition += 0.1;
            }
            else if(gamepad1.left_trigger > 0){
                intakePosition -= 0.1;
            }

            robot.intakeR.setPosition(intakePosition);
            robot.intakeL.setPosition(intakePosition);

            if(gamepad1.right_bumper) {
                robot.lift.setPower(1);
            } else if (gamepad1.left_bumper){
                robot.lift.setPower(-1);
            } else{
                robot.lift.setPower(0);
            }

        }

    }

}



