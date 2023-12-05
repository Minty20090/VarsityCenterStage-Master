package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "GyroTest")
public class GyroTest extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);



        while (opModeIsActive()) {


        }

    }

}



