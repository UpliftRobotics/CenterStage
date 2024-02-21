package org.firstinspires.ftc.teamcode.Core.programs.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;

@Autonomous(name = "IntakeTest", group = "Opmodes")
public class IntakeTest extends UpliftAutoImpl {

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

        robot.getIntakeRoller().setPosition(robot.frontRollerStore);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightReset);
        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftReset);

        Thread.sleep(3000);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);

        Thread.sleep(3000);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);

        Thread.sleep(3000);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);

        Thread.sleep(3000);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);

        Thread.sleep(3000);


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
//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//        timer.startTime();
//
//        odom.setOdometryPosition(0, 0, 0);
//
//
//        while(timer.seconds() < 25)
//        {
//            //do auto
//        }
//        //park

        intake(0.7);

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
