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

    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException
    {
//        while(robot.getPixelDetectorLeft().getDistance(DistanceUnit.INCH) < 1)
//        {
//            robot.getIntake().setPower(0.1);
//        }

        robot.getGrabber().setPosition(robot.grabberOpen);

        robot.getTwister().setPosition(robot.twisterPos4);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
        robot.getIntakeRoller().setPosition(robot.frontRollerGround);
//
        robot.getArmLeft().setPosition(robot.armLeftStore);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getDepositWrist().setPosition(robot.depositWristStore);

        Thread.sleep(1000);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftHold);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightHold);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);

        Thread.sleep(1000);

        robot.getArmLeft().setPosition(robot.armLeftTransfer);
        robot.getArmRight().setPosition(robot.armRightTransfer);
        robot.getDepositWrist().setPosition(robot.depositWristTransfer);

        Thread.sleep(1000);

        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);

        Thread.sleep(1000);

        robot.getGrabber().setPosition(robot.grabberClose2);
        Thread.sleep(1000);
//
//
////        robot.getArmLeft().setPosition(robot.armLeftStore);
////        robot.getArmRight().setPosition(robot.armRightStore);
////        robot.getDepositWrist().setPosition(robot.depositWristStore);
//
//        Thread.sleep(1000);
//
//
//        claw("open");
//
//        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);
//        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
//        robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//
//        Thread.sleep(5000);
//
//
//        Thread.sleep(1000);
//
//
//
//        Thread.sleep(5000);
////
////        claw("close2");
////
////        Thread.sleep(1000);


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
