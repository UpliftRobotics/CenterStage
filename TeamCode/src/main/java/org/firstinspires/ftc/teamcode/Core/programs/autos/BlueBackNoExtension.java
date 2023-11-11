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
        robot.getIntakeAngleRight().setPosition(robot.intakeStorePos);
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
            robot.getGrabber().setPosition(robot.grabberOpen);

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
            robot.getGrabber().setPosition(robot.grabberOpen);

            //outtake position
            driveToPosition(30, 30, 0.5, 93);
        }

        // right

        if(location == 2 )
        {
            //drop position
            driveToPosition(4, 27, 0.6, 90);
            Thread.sleep(1000);

            deposit();
            Thread.sleep(500);
            robot.getGrabber().setPosition(robot.grabberOpen);

            Thread.sleep(1000);

            //outtake position
            driveToPosition(42, 18, 0.5, 93);

        }

        drop();
        Thread.sleep(1000);

        robot.getIntakeAngleRight().setPosition(robot.intakeGroundPos);
        intake(-0.2);
        Thread.sleep(5000);
        intake(0);



        //park
        driveToPosition(10, 44, 0.6, 93);
        driveToPosition(2, 47, 0.6, 100);


    }

    @Override
    public void exit() throws InterruptedException {

    }
}
