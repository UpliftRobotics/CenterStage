package org.firstinspires.ftc.teamcode.Core.programs.autos;

import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.QUICKEST_DIRECTION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;

@Autonomous(name = "TestAuto", group = "Opmodes")
public class TestAuto extends UpliftAutoImpl {

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
        odom.setOdometryPosition(0, 0, 0);

        driveToPosition(0, 48, 0.6, 105);

        Thread.sleep(20000);
//        driveToPosition(0, 12, 0.5, 0 );

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
