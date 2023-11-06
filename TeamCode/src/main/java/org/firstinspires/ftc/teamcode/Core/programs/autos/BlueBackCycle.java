package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;

@Autonomous(name = "BlueBackCycle", group = "Opmodes")
public class BlueBackCycle extends UpliftAutoImpl
{
    Odometry odom;

    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() {
//        robot = new UpliftRobot(this);
    }

    @Override
    public void body() throws InterruptedException
    {
        odom.setOdometryPosition(48, 0, 180);

        //left

        //drop position
        driveToPosition(4, 10, 0.6, 90);
        Thread.sleep(1000);

        //extension position
        driveToPosition(15, 27, 0.5, 93);
        Thread.sleep(1000);




//        //middle
//
//        //drop position
//        driveToPosition(4, 18, 0.6, 90);
//        Thread.sleep(1000);
//
//
//        //extension position
//        driveToPosition(15, 27, 0.5, 93);
//
//
//
//        // right
//
//        //drop position
//        driveToPosition(4, 25, 0.6, 90);
//        Thread.sleep(1000);
//
//        //extension position
//        driveToPosition(15, 27, 0.5, 93);

        //drive to bridge

        driveToPosition(20, 40, 0.6, 100);

        driveToPosition(72, 30, 0.6, 107);
        Thread.sleep(1000);

        //drive back to board

        driveToPosition(20, 30, 0.6, 107);
        Thread.sleep(1000);

        //drop position
        driveToPosition(4, 25, 0.6, 90);
        Thread.sleep(1000);




        Thread.sleep(20000);

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
