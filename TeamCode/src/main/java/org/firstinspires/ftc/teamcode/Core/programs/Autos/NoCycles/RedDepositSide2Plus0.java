package org.firstinspires.ftc.teamcode.Core.programs.Autos.NoCycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Red Deposit Side 2 + 0", group = "Opmodes")
public class RedDepositSide2Plus0 extends UpliftAutoImpl
{
    Odometry odom;

    public void initHardware()
    {
        robot = new UpliftRobot(this);
        odom = robot.odometry;
    }

    @Override
    public void initAction() throws InterruptedException {
        robot.getGrabber().setPosition(robot.grabberClose1);

        Thread.sleep(2000);

        robot.getArmLeft().setPosition(0.7);
        robot.getArmRight().setPosition(0.3);

        robot.getDepositWrist().setPosition(0.6);

        robot.webcam.setPipeline(robot.pipelineRedDepositSide);
    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineRedDepositSide.location;
        odom.setOdometryPosition(48, 144, 0);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(30, 135, 0.8, 0);
            driveToPosition(4, 119, 0.7, 90);
            Thread.sleep(1000);

            deposit(400, 0.1);
            Thread.sleep(500);


            claw("open");
            Thread.sleep(1000);
            reset(true, false);


            //outtake position
            driveToPosition(43, 124, 0.5, 95);

        }

        //middle
        if(location == 1 )
        {
            //drop position
            driveToPosition(30, 135, 0.8, 0);
            driveToPosition(4, 125, 0.7, 90);
            Thread.sleep(1000);


            deposit(400, 0.1);
            Thread.sleep(500);

            claw("open");
            Thread.sleep(1000);
            reset(true, false);


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

            deposit(400, 0.1);
            Thread.sleep(500);

            claw("open");
            Thread.sleep(1000);
            reset(true, false);


            //outtake position
            driveToPosition(24, 118, 0.5, 90);

        }

        Thread.sleep(1000);

        intake(-0.175);
        Thread.sleep(5000);
        intake(0);



        //park
        driveToPosition(6, 152, 0.6, 90);
//        reset();

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
