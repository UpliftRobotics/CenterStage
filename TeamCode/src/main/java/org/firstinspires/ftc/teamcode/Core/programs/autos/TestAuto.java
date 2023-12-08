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
    public void initAction() throws InterruptedException
    {

    }

    @Override
    public void body() throws InterruptedException
    {

        robot.opMode.telemetry.addData("left slide pos", robot.getSlideLeft().getCurrentPosition());
        robot.opMode.telemetry.addData("right slide pos", robot.getSlideRight().getCurrentPosition());
        robot.opMode.telemetry.update();

        odom.setOdometryPosition(0, 0, 0);

        deposit(600, 0.1,true );
        Thread.sleep(5000);

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
