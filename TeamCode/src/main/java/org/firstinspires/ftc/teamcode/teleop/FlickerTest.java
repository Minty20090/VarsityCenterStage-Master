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

        while (opModeIsActive()) {

            // Teleop Code goes here
            if(gamepad1.a){
                robot.door.setPosition(1);
            }

            if(gamepad1.b){
                robot.door.setPosition(0);
            }

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



