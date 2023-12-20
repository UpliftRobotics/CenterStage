package org.firstinspires.ftc.teamcode.Core.programs.Autos.Cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Blue Deposit Side Cycle", group = "Opmodes")
public class BlueAudienceSideCycle extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException {

        robot.getGrabber().setPosition(robot.grabberClosePos);

        Thread.sleep(2000);

        robot.getArmLeft().setPosition(0.7);
        robot.getArmRight().setPosition(0.3);

        robot.getDepositWrist().setPosition(0.6);

        robot.webcam.setPipeline(robot.pipelineBlueDepositSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlueDepositSide.location;
        odom.setOdometryPosition(48, 0, 180);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(30, 6, 0.8, 180);
            driveToPosition(3.5, 10, 0.7, 90, 2);
            Thread.sleep(1000);



            deposit(400, 0.1);
            Thread.sleep(500);


            drop();
            Thread.sleep(1000);
            reset();


            //outtake position
            driveToPosition(23, 21, 0.5, 93);

        }

        //middle
        if(location == 1 )
        {
            //drop position
            driveToPosition(30, 6, 0.8, 180);
            driveToPosition(3.5, 16, 0.7, 90, 2);
            Thread.sleep(1000);


            deposit(400, 0.1);
            Thread.sleep(500);

            drop();
            Thread.sleep(1000);
            reset();


            //outtake position
            driveToPosition(28, 28, 0.5, 93);
        }

        // right

        if(location == 2 )
        {
            //drop position
            driveToPosition(30, 6, 0.8, 180);
            driveToPosition(3.5, 25, 0.7, 90, 2);
            Thread.sleep(1000);

            deposit(400, 0.1);
            Thread.sleep(500);

            drop();
            Thread.sleep(1000);
            reset();

            //outtake position
            driveToPosition(41, 19, 0.5, 93);

        }

        Thread.sleep(1000);

        intake(-0.175);
        Thread.sleep(5000);
        intake(0);


        //park
        driveToPosition(5, -10, 0.6, 90);




    }

    @Override
    public void exit() throws InterruptedException {

    }
}
