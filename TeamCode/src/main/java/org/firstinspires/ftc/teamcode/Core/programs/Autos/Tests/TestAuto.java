package org.firstinspires.ftc.teamcode.Core.programs.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;

@Autonomous(name = "TestAuto", group = "Opmodes")
public class TestAuto extends UpliftAutoImpl {

    Odometry odom;

    public void initHardware()
    {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException
    {

    }

    @Override
    public void body() throws InterruptedException
    {
        odom.setOdometryPosition(48, 0, 0);
        driveToPosition(42, 28, 0.9, 0);
        Thread.sleep(10000);


    }

    @Override
    public void exit() throws InterruptedException {

    }
}
