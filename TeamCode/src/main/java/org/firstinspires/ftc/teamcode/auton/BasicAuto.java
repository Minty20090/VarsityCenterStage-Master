package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
@Autonomous(name = "BasicAuto")
public class BasicAuto extends LinearOpMode{

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);

        while(!isStarted()){
            // only the ghost of programmers past know why tf this is set up this way
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
        }
        waitForStart(); //wait for play button to be pressed

        // Autonomous code starts here
        robot.fLeftWheel.setPower(1);
        robot.fRightWheel.setPower(1);
        robot.bLeftWheel.setPower(1);
        robot.bRightWheel.setPower(1);
        sleep(1000);
        robot.fLeftWheel.setPower(0);
        robot.fRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);

    }
}
