package org.firstinspires.ftc.teamcode.Core.programs.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;

@Autonomous(name = "RollerTest", group = "Opmodes")
public class RollerTest extends UpliftAutoImpl {

    Odometry odom;

    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException
    {
        robot.getIntakeRoller().setPosition(robot.frontRollerGround);
        Thread.sleep(2000);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);



       }

    @Override
    public void body() throws InterruptedException
    {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();

        odom.setOdometryPosition(0, 0, 0);


        while(timer.seconds() < 25)
        {
            //do auto
        }
        //park




        intake(-0.175);

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
