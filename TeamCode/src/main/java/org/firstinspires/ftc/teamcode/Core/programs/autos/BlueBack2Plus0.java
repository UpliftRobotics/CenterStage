package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "BlueBack2+0", group = "Opmodes")
public class BlueBack2Plus0 extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException {

//        robot.getGrabberLeft().setPosition(robot.grabberLeftClose);
        robot.getGrabberRight().setPosition(robot.grabberRightClose);

        Thread.sleep(2000);

        robot.getArmLeft().setPosition(robot.armLeftHold);
        robot.getArmRight().setPosition(robot.armRightHold);

        robot.getDepositWrist().setPosition(0.5);

    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlue.location;
        odom.setOdometryPosition(48, 0, 180);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(30, 6, 0.8, 180);
            driveToPosition(3, 10, 0.7, 90, 2);
            Thread.sleep(1000);



            deposit(200, 0.1);
            Thread.sleep(500);


            drop();
            Thread.sleep(1000);

            //outtake position
            driveToPosition(23, 21, 0.5, 93);

        }

        //middle
        if(location == 1 )
        {
            //drop position
            driveToPosition(30, 6, 0.8, 180);
            driveToPosition(3, 16, 0.7, 90, 2);
            Thread.sleep(1000);


            deposit(200, 0.1);
            Thread.sleep(500);

            drop();
            Thread.sleep(1000);

            //outtake position
            driveToPosition(28, 28, 0.5, 93);
        }

        // right

        if(location == 2 )
        {
            //drop position
            driveToPosition(30, 6, 0.8, 180);
            driveToPosition(3, 25, 0.7, 90, 2);
            Thread.sleep(1000);

            deposit(200, 0.1);
            Thread.sleep(500);

            drop();
            Thread.sleep(1000);

            //outtake position
            driveToPosition(41, 19, 0.5, 93);

        }

        Thread.sleep(1000);

        intake(-0.175);
        Thread.sleep(5000);
        intake(0);


        //park
        driveToPosition(5, -10, 0.6, 90);
        reset();




    }

    @Override
    public void exit() throws InterruptedException {

    }
}
