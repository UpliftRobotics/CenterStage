package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;

@Autonomous(name = "TestAuto", group = "Opmodes")
public class TestAuto extends UpliftAutoImpl {

    Odometry odom;

    public void initHardware() {

    }

    @Override
    public void initAction() {

    }


    @Override
    public void body() throws InterruptedException {
        odom.setOdometryPosition(0, 0, 0);
        goToPos(0, 5, 0, 0.2, 0.2);
    }

    @Override
    public void exit() throws InterruptedException {

    }
}
