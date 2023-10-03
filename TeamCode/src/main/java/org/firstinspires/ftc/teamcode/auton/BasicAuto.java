package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.exception.RobotCoreException;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
@Autonomous(name = "BasicAuto")
enum Side {
    rBlue,
    lBlue,
    rRed,
    lRed
}
public class BasicAuto extends LinearOpMode{

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
        Side c = Side.rBlue;
        int side = 1;
        if(gamepad1.right_bumper == true){
            if(side<4) {
                side++;
            }
            else if(side == 4){
                side = 1;
            }
        }
        switch(side){
            case 1:c = Side.rBlue;
            break;
            case 2:c = Side.lBlue;
                robot.fRightWheel.setPower(1);
                robot.fLeftWheel.setPower(-1);
                robot.bRightWheel.setPower(-1);
                robot.bLeftWheel.setPower(1);
                sleep(1000);
                robot.fRightWheel.setPower(0);
                robot.fLeftWheel.setPower(0);
                robot.bRightWheel.setPower(0);
                robot.bLeftWheel.setPower(0);
            break;
            case 3:c = Side.rRed;
                robot.fRightWheel.setPower(-1);
                robot.fLeftWheel.setPower(1);
                robot.bRightWheel.setPower(1);
                robot.bLeftWheel.setPower(-1);
                sleep(1000);
                robot.fRightWheel.setPower(0);
                robot.fLeftWheel.setPower(0);
                robot.bRightWheel.setPower(0);
                robot.bLeftWheel.setPower(0);
            break;
            case 4:c = Side.lRed;
            break;
        }

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
