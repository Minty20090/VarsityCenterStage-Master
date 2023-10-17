package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Projects.FleaFlickerMap;
import com.qualcomm.robotcore.exception.RobotCoreException;
import org.firstinspires.ftc.teamcode.Projects.HWMap;
import org.firstinspires.ftc.teamcode.auton.OpenCV;
@Autonomous(name = "BasicAuto")

public class BasicAuto extends LinearOpMode{

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();


    public HWMap robot = new HWMap();

    @Override
    public void runOpMode() throws InterruptedException {
       // enum Side {
           // rBlue,
          //  lBlue,
           // rRed,
          //  lRed
        //}


        //initialize hardware map
        robot.init(hardwareMap);
       // Side c = Side.rBlue;
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
            case 1:;
            break;
            case 2:
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
            case 3:
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
            case 4:

            break;
        }

        while(!isStarted()){
            // only the ghost of programmers past know why tf this is set up this way
            //ok boomer
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
    public void forward (double power, int time){

        robot.fLeftWheel.setPower(power);
        robot.fRightWheel.setPower(power);
        robot.bLeftWheel.setPower(power);
        robot.bRightWheel.setPower(power);
        sleep(time);
        robot.fLeftWheel.setPower(0);
        robot.fRightWheel.setPower(0);
        robot.bLeftWheel.setPower(0);
        robot.bRightWheel.setPower(0);


    }
    public void turn(int time, double direction){
        robot.fLeftWheel.setPower(direction);
        robot.fRightWheel.setPower(-direction);
        robot.bLeftWheel.setPower(direction);
        robot.bRightWheel.setPower(-direction);
        sleep(time);
    }
    public void drop(){
       // robot.lift.setPosition(0);
       // robot.gate.setPosition(0);
    }

    public void spike(String location) {
        if (location == "Middle") {
            System.out.println("bet");
            forward(.8,1000);
            //arm stuff
        }
        else if(location == "Right"){
            forward(.8,1000);
            turn(1,-.8);
            //arm stuff
        }
        else if(location == "Left"){
            forward(.8,1000);
            turn(1,-.8);
            //arm stuff
        }
    }
}


