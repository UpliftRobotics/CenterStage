package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "BlueBackNoExtension", group = "Opmodes")

public class BlueBackNoExtension extends UpliftAutoImpl
{
    Odometry odom;

    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() {
//        robot = new UpliftRobot(this);
        robot.getGrabber().setPosition(robot.grabberClose);
        robot.getDepositArm().setPosition(0.5);
        robot.getDepositWrist().setPosition(0.3);
        robot.getDepositTwist().setPosition(0.3);
    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlue.location;
        odom.setOdometryPosition(48, 0, 180);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(4, 12, 0.6, 90);
            Thread.sleep(1000);

            deposit();
            Thread.sleep(500);
            drop();

            //outtake position
            driveToPosition(23, 20, 0.5, 93);

        }

        //middle
        if(location == 1 )
        {
            //drop position
            driveToPosition(4, 18, 0.6, 90);
            Thread.sleep(1000);

            deposit();
            Thread.sleep(500);
            drop();

            //outtake position
            driveToPosition(30, 27, 0.5, 93);
        }

        // right

        if(location == 2 )
        {
            //drop position
            driveToPosition(4, 25, 0.6, 90);
            Thread.sleep(1000);

            deposit();
            Thread.sleep(500);
            drop();

            Thread.sleep(1000);

            //outtake position
            driveToPosition(42, 16, 0.5, 93);
        }

        intake(-1);
        Thread.sleep(5000);



        //park
        driveToPosition(10, 42, 0.6, 93);
        driveToPosition(0, 46, 0.6, 93);


    }

    @Override
    public void exit() throws InterruptedException {

    }
}
