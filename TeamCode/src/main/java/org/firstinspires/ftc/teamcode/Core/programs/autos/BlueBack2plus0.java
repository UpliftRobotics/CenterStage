package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "BlueBack2plus0", group = "Opmodes")
public class BlueBack2plus0 extends UpliftAutoImpl
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
        int location = robot.pipelineBlue.location;
        odom.setOdometryPosition(48, 0, 180);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(4, 10, 0.6, 90);
            Thread.sleep(1000);

        }

        //middle
        if(location == 1 ) {

            //drop position
            driveToPosition(4, 18, 0.6, 90);
            Thread.sleep(1000);

        }

        // right

        if(location == 2 ) {
            //drop position
            driveToPosition(4, 25, 0.6, 90);
            Thread.sleep(1000);
        }

        //park
        //extension position
        driveToPosition(15, 27, 0.5, 93);


        driveToPosition(10, 42, 0.6, 93);
        driveToPosition(0, 44, 0.6, 93);


    }

    @Override
    public void exit() throws InterruptedException {

    }
}
