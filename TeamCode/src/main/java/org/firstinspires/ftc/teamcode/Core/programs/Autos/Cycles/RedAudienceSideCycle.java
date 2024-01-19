package org.firstinspires.ftc.teamcode.Core.programs.Autos.Cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Red Audience Side Cycle", group = "Opmodes")
public class RedAudienceSideCycle extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException {

//        robot.getGrabber().setPosition(robot.grabberClose1);
//
//        Thread.sleep(2000);
//
//        robot.getArmLeft().setPosition(0.7);
//        robot.getArmRight().setPosition(0.3);
//
//        robot.getDepositWrist().setPosition(0.6);

        robot.webcam.setPipeline(robot.pipelineRedAudienceSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineRedAudienceSide.location;
        odom.setOdometryPosition(100, 144, 180);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(110, 116, 0.7, 180);

            driveToPosition(108, 129, 0.7, 180);
            driveToPosition(96, 120, 0.8, 180);
            driveToPosition(96, 90, 0.7, 180);
            driveToPosition(96, 85, 0.7, 90, 2);

            Thread.sleep(1000);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);

            Thread.sleep(1000);

            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            driveToPosition(111, 87, 0.7, 90);

            Thread.sleep(1000);

            robot.getIntakeRoller().setPosition(robot.frontRollerGround);

            intake(.5);








//            driveToPosition(3.5, 10, 0.7, 90, 2);
//            Thread.sleep(1000);
//
//
//
//            deposit(400, 0.1);
//            Thread.sleep(500);
//
//
//
//            claw("open");
//            Thread.sleep(1000);
//            reset(true, false);
//
//
//            //outtake position
//            driveToPosition(23, 21, 0.5, 93);

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


            claw("open");
            Thread.sleep(1000);
            reset(true, false);


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


            claw("open");
            Thread.sleep(1000);
            reset(true, false);

            //outtake position
            driveToPosition(41, 19, 0.5, 93);

        }

//        Thread.sleep(1000);
//
//        intake(-0.175);
//        Thread.sleep(5000);
//        intake(0);
//
//
//        //park
//        driveToPosition(5, -10, 0.6, 90);




    }

    @Override
    public void exit() throws InterruptedException {

    }
}
