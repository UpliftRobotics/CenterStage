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
    public void initAction() {
//        robot = new UpliftRobot(this);
    }

    @Override
    public void body() throws InterruptedException
    {
        robot.getDepositArm().setPosition(robot.depositHold);
        robot.getDepositWrist().setPosition(robot.wristHold);
        robot.getIntakeAngleRight().setPosition(robot.intakeStorePos);
        robot.getGrabber().setPosition(robot.grabberOpen);
        Thread.sleep(2000);
        robot.getDepositWrist().setPosition(robot.wristPick);
        Thread.sleep(2000);
        robot.getDepositArm().setPosition(robot.depositPick);
        Thread.sleep(2000);
        robot.getDepositWrist().setPosition(robot.wristPick2);
        Thread.sleep(2000);
        robot.getGrabber().setPosition(robot.grabberClose);
        Thread.sleep(2000);
        robot.getIntakeAngleRight().setPosition(robot.intake4Pixel);
        Thread.sleep(2000);
        robot.getDepositArm().setPosition(robot.depositHold);
        Thread.sleep(2000);

        Thread.sleep(200000);
//        robot.getIntakeAngleRight().setPosition(robot.intakeGroundPos);
//        Thread.sleep(2000);
//        robot.getIntakeAngleRight().setPosition(robot.intake2Pixel);
//        Thread.sleep(2000);
//        robot.getIntakeAngleRight().setPosition(robot.intake3Pixel);
//        Thread.sleep(2000);
//        robot.getIntakeAngleRight().setPosition(robot.intake4Pixel);
//        Thread.sleep(2000);
//        robot.getIntakeAngleRight().setPosition(robot.intake5Pixel);
//        Thread.sleep(2000);
//        robot.getIntakeAngleRight().setPosition(robot.intakeStorePos);
//        Thread.sleep(2000);

//        robot.getDepositArm().setPosition(robot.depositBack);
//        robot.getDepositWrist().setPosition(robot.wristBack);
//        robot.getGrabber().setPosition(robot.grabberClose);
//        Thread.sleep(5000);
//        robot.getDepositArm().setPosition(robot.depositHold);
//        robot.getDepositWrist().setPosition(robot.wristHold);
//        robot.getGrabber().setPosition(robot.grabberOpen);
//        Thread.sleep(5000);
//        robot.getDepositArm().setPosition(robot.depositPick);
//        robot.getDepositWrist().setPosition(robot.wristPick);
//        Thread.sleep(5000);

//        odom.setOdometryPosition(0, 0, 0);
//
//        driveToPosition(0, 48, 0.6, 105);
//
//        Thread.sleep(20000);
////        driveToPosition(0, 12, 0.5, 0 );

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
