package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "RedBackNoExtension", group = "Opmodes")
public class RedBackNoExtension extends UpliftAutoImpl
{
    Odometry odom;

    public void initHardware()
    {
        robot = new UpliftRobot(this);
        odom = robot.odometry;
    }

    @Override
    public void initAction()
    {
        robot.getGrabberLeft().setPosition(robot.grabberLeftClose);
        robot.getArmLeft().setPosition(robot.armLeftTransfer);
        robot.getArmRight().setPosition(robot.armRightTransfer);
        robot.getDepositWrist().setPosition(robot.depositWristTransfer);

        robot.getPlane().setPosition(1);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineRed.location;
        odom.setOdometryPosition(48, 144, 0);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(9, 110, 0.6, 82);
            Thread.sleep(1000);

            deposit(500, 0.1);
            Thread.sleep(500);


            drop();
            Thread.sleep(1000);


            //outtake position
            driveToPosition(46, 124, 0.5, 95);

        }

        //middle
        if(location == 1 )
        {
            //drop position
            driveToPosition(6, 113, 0.6, 85);
            Thread.sleep(1000);


            deposit(500, 0.1);
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
            driveToPosition(8, 125, 0.6, 85);
            Thread.sleep(1000);

            deposit(500, 0.1);
            Thread.sleep(500);

            drop();
            Thread.sleep(1000);

            //outtake position
            driveToPosition(26, 118, 0.5, 85);

        }

        drop();
        Thread.sleep(1000);

        intake(-0.2);
        Thread.sleep(5000);
        intake(0);
//
//
//
        //park
        driveToPosition(10, 95, 0.6, 93);
        driveToPosition(7, 95, 0.6, 85);


    }

    @Override
    public void exit() throws InterruptedException {

    }
}
