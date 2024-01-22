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

        claw("open");

        robot.getDepositWrist().setPosition(robot.depositWristStore);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);

        robot.getTwister().setPosition(robot.twisterPos4);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getArmLeft().setPosition(robot.armLeftStore);

        Thread.sleep(1000);

//        claw("close1");

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
            driveToPosition(112, 87, 0.7, 90);

            Thread.sleep(1000);

            robot.getIntakeRoller().setPosition(robot.frontRollerGround);

            intake(.5);
            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntake().setPower(.5);
            Thread.sleep(1000);
            robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
            robot.getArmLeft().setPosition(robot.armLeftTransfer);
            robot.getArmRight().setPosition(robot.armRightTransfer);
            Thread.sleep(500);
            robot.getGrabber().setPosition(robot.grabberClose2);
            robot.getIntake().setPower(0);

            driveToPosition(70, 90, .7,90); // under bridge
            driveToPosition(30, 90, .7,90);


            driveToPosition(4, 106, 0.8, 78);
            Thread.sleep(1000);


            robot.getArmLeft().setPosition(robot.armLeftDrop);
            robot.getArmRight().setPosition(robot.armRightDrop);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
            Thread.sleep(100);
            robot.getDepositWrist().setPosition(robot.depositWristDrop);

            Thread.sleep(1000);

            claw("open");
            Thread.sleep(1000);

            driveToPosition(10, 115, 0.8, 78);








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
            driveToPosition(100, 113, 0.8, 190);
            driveToPosition(114, 122, 0.7, 182);

            driveToPosition(111, 97, 0.7, 180);
            driveToPosition(115, 90,  0.7, 90, 2);


            Thread.sleep(1000);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);

            Thread.sleep(1000);


            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            driveToPosition(118, 93, 0.7, 80);

            Thread.sleep(1000);

            robot.getIntakeRoller().setPosition(robot.frontRollerGround);

            intake(.5);

            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntake().setPower(.5);
            Thread.sleep(1000);
            robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
            robot.getArmLeft().setPosition(robot.armLeftTransfer);
            robot.getArmRight().setPosition(robot.armRightTransfer);
            Thread.sleep(500);
            robot.getGrabber().setPosition(robot.grabberClose2);
            robot.getIntake().setPower(0);

            driveToPosition(70, 85, .7,90); // under bridge
            driveToPosition(30, 85, .7,90);



            driveToPosition(6, 109, 0.8, 78);
            Thread.sleep(1000);


            robot.getArmLeft().setPosition(robot.armLeftDrop);
            robot.getArmRight().setPosition(robot.armRightDrop);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
            Thread.sleep(100);
            robot.getDepositWrist().setPosition(robot.depositWristDrop);

            Thread.sleep(1000);

            claw("open");
            Thread.sleep(1000);

            driveToPosition(10, 120, 0.8, 78);


//
//
//            deposit(400, 0.1);
//            Thread.sleep(500);
//
//
//            claw("open");
//            Thread.sleep(1000);
//            reset(true, false);
//
//
//            //outtake position
//            driveToPosition(28, 28, 0.5, 93);
        }

        // right

        if(location == 2)
        {
            // odom.setOdometryPosition(100, 144, 180);
            //drop position
            driveToPosition(100, 125, 0.8, 180);

            driveToPosition(90, 115, 0.8, 215);
            driveToPosition(110, 125, 0.8, 215);
            Thread.sleep(1000);
            driveToPosition(110, 85, 0.9, 95,2);
            Thread.sleep(1000);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);

            Thread.sleep(1000);


            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            driveToPosition(112.5, 85, 0.7, 92);

            Thread.sleep(1000);

            robot.getIntakeRoller().setPosition(robot.frontRollerGround);

            intake(.5);
            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntake().setPower(.5);
            Thread.sleep(1000);
            robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
            robot.getArmLeft().setPosition(robot.armLeftTransfer);
            robot.getArmRight().setPosition(robot.armRightTransfer);
            Thread.sleep(500);
            robot.getGrabber().setPosition(robot.grabberClose2);
            robot.getIntake().setPower(0);

            driveToPosition(70, 90, .7,90); // under bridge
            driveToPosition(30, 90, .7,90);



            driveToPosition(4.5, 128, 0.8, 78);
            Thread.sleep(1000);


            robot.getArmLeft().setPosition(robot.armLeftDrop);
            robot.getArmRight().setPosition(robot.armRightDrop);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
            Thread.sleep(100);
            robot.getDepositWrist().setPosition(robot.depositWristDrop);

            Thread.sleep(1000);

            claw("open");
            Thread.sleep(1000);

            driveToPosition(10, 125, 0.8, 78);


        }

        robot.getArmLeft().setPosition(robot.armLeftStore);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getDepositWrist().setPosition(robot.depositWristStore);
        robot.getTwister().setPosition(robot.twisterPos4);
        Thread.sleep(1000);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);
        Thread.sleep(1000);
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
