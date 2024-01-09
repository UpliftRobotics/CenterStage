package org.firstinspires.ftc.teamcode.Core.programs.Autos.Cycles;

import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.blueLeftStack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;


@Autonomous(name = "Blue Audience Side Cycle", group = "Opmodes")
public class BlueAudienceSideCycle extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware() {

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

        robot.webcam.setPipeline(robot.pipelineBlueDepositSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlueDepositSide.location;
        odom.setOdometryPosition(48, 0, 180);

        if(!goPark)
        {
            //left
            if(location == 0 || location == -1 )
            {
                //drop position
                driveToPosition(30, 6, 0.8, 180);
                driveToPosition(3.5, 10, 0.7, 90, 2);
                Thread.sleep(1000);



                deposit(400, 0.1);
                Thread.sleep(500);


//                drop();
                Thread.sleep(1000);
                reset(true, false);


                //outtake position
                driveToPosition(10, 15, 0.5, 93);

                //extend to outtake
                extensionPID(400, 200, 0.5);

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

//                drop();
                Thread.sleep(1000);
                reset(true, false);


                //outtake position
                driveToPosition(10, 20, 0.5, 93);

                //extend to outtake
                extensionPID(500, 250, 0.5);
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

//                drop();
                Thread.sleep(1000);
                reset(true, false);

                //outtake position
                driveToPosition(10, 15, 0.5, 93);

                //extend to outtake
                extensionPID(600, 300, 0.5);

            }

            Thread.sleep(100);
            intake(-0.175);

            reset(false, true);

            cycles("blue", 2);
        }


        //park

        driveToPosition(10, -10, 0.6, 90);
        driveToPosition(5, -10, 0.6, 90);




    }

    @Override
    public void exit() throws InterruptedException {

    }
}
