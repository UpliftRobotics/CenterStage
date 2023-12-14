package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Red Audience Side 2 + 0", group = "Opmodes")
public class RedAudienceSide2Plus0 extends UpliftAutoImpl
{
    Odometry odom;

    public void initHardware()
    {
        robot = new UpliftRobot(this);
        odom = robot.odometry;
    }

    @Override
    public void initAction() throws InterruptedException {
        robot.getGrabberLeft().setPosition(robot.grabberLeftClose);
//        robot.getGrabberRight().setPosition(robot.grabberRightClose);

        Thread.sleep(2000);

        robot.getArmLeft().setPosition(robot.armLeftHold);
        robot.getArmRight().setPosition(robot.armRightHold);

        robot.getDepositWrist().setPosition(0.5);

        robot.webcam.setPipeline(robot.pipelineRedAudienceSide);

    }

    @Override
    public void body() throws InterruptedException
    {
        int wait = 2000;
        int location = robot.pipelineRedAudienceSide.location;
        odom.setOdometryPosition(100, 144, 0);

        //left
        if(location == 0 || location == -1 ) {
            //outtake position
            driveToPosition(100, 95, 0.5, 0);
            Thread.sleep(1000);

            driveToPosition(103, 98, 0.5, 22.5);
            Thread.sleep(1000);


            intake(-0.175);
            Thread.sleep(3000);
            intake(0);


//            driveToPosition(100, 95, 0.5, 22.5);
//            Thread.sleep(200);
//
//            driveToPosition(100, 95, 0.5, 85);
//            Thread.sleep(1000);
//
//            driveToPosition(8, 89, 0.5, 85);
//            Thread.sleep(wait);
//            deposit(400,.1, true);
//            Thread.sleep(1000);
//
//            driveToPosition(5.5, 103, 0.5, 85);
//            Thread.sleep(1000);
//
//            drop();
//
//            Thread.sleep(1000);
//
//            reset();
//            driveToPosition(7, 89, 0.5, 85);
//            Thread.sleep(1000);
//            deposit(10, .01, false);
//            Thread.sleep(1000);


        }

        //middle
        if(location == 1 )
        {
//            //drop position
            driveToPosition(100, 95, 0.5, 0);
            Thread.sleep(1000);

            driveToPosition(100, 98, 0.5, 0);
            Thread.sleep(1000);

            intake(-0.175);
            Thread.sleep(3000);
            intake(0);

//            driveToPosition(100, 95, 0.5, 0);
//            Thread.sleep(1000);
//
//            driveToPosition(100, 95, 0.5, 85);
//            Thread.sleep(1000);
//
//            driveToPosition(8, 89, 0.5, 85);
//            Thread.sleep(wait);
//            deposit(400,.1, true);
//            Thread.sleep(1000);
//
//            driveToPosition(5, 115, 0.5, 85);
//            Thread.sleep(1000);
//
//            drop();
//            Thread.sleep(1000);
//
//            reset();
//            driveToPosition(7, 89, 0.5, 85);
//            Thread.sleep(1000);
//            deposit(10, .01, false);
//            Thread.sleep(1000);


        }

        // right

        if(location == 2 )
        {
//           //drop position
            driveToPosition(100, 120, 0.7, 0);
            Thread.sleep(1000);

            driveToPosition(105, 115, 0.7, -90, 2);
            Thread.sleep(1000);

            intake(-0.2);
            Thread.sleep(3000);
            intake(0);

//            driveToPosition(115, 115, 0.7, -90);
//            Thread.sleep(1000);
//
//            driveToPosition(115, 100, 0.7, -90);
//            Thread.sleep(1000);
//
//            driveToPosition(10, 110, 0.7, 90);
//            Thread.sleep(wait);
//
//            driveToPosition(5, 120, 0.7, 85);
//            Thread.sleep(1000);
//
//
//            deposit(400,.1, true);
//            Thread.sleep(1000);
//
//            drop();
//            Thread.sleep(1000);
//
//            reset();
//            driveToPosition(7, 89, 0.7, 85);
//            Thread.sleep(1000);
//            deposit(10, .01, false);
//            Thread.sleep(1000);






        }





//        //park
//        driveToPosition(5, 152, 0.6, 90);
//        reset();

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
