package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "RedBack2+0", group = "Opmodes")
public class RedBack2Plus0 extends UpliftAutoImpl
{
    Odometry odom;

    public void initHardware()
    {
        robot = new UpliftRobot(this);
        odom = robot.odometry;
    }

    @Override
    public void initAction() throws InterruptedException {
        robot.getGrabberLeft().setPosition(robot.grabberLeftClose);
//        robot.getGrabberRight().setPosition(robot.grabberRightClose);

        Thread.sleep(2000);

        robot.getArmLeft().setPosition(robot.armLeftHold);
        robot.getArmRight().setPosition(robot.armRightHold);

        robot.getDepositWrist().setPosition(0.5);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineRedClose.location;
        odom.setOdometryPosition(48, 144, 0);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(30, 135, 0.8, 0);
            driveToPosition(3, 119, 0.7, 90);
            Thread.sleep(1000);

            deposit(400, 0.1,true );
            Thread.sleep(500);


            drop();
            Thread.sleep(1000);


            //outtake position
            driveToPosition(43, 124, 0.5, 95);

        }

        //middle
        if(location == 1 )
        {
            //drop position
            driveToPosition(30, 135, 0.8, 0);
            driveToPosition(3, 125, 0.7, 90);
            Thread.sleep(1000);


            deposit(400, 0.1,true );
            Thread.sleep(500);

            drop();
            Thread.sleep(1000);

            //outtake position
            driveToPosition(33, 115, 0.5, 85);
        }

        // right

        if(location == 2 )
        {
            //drop position
            driveToPosition(30, 130, 0.8, 0);
            driveToPosition(4.5, 135, 0.7, 90);
            Thread.sleep(1000);

            deposit(400, 0.1,true );
            Thread.sleep(500);

            drop();
            Thread.sleep(1000);

            //outtake position
            driveToPosition(24, 118, 0.5, 90);

        }

        Thread.sleep(1000);

        intake(-0.175);
        Thread.sleep(5000);
        intake(0);



        //park
        driveToPosition(5, 152, 0.6, 90);
        reset();

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
