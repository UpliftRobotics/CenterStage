package org.firstinspires.ftc.teamcode.Core.programs.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;

@Autonomous(name = "TransferTest", group = "Opmodes")
public class TransferTest extends UpliftAutoImpl {

    Odometry odom;

    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException
    {
        robot.getArmLeft().setPosition(robot.armLeftStore);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
        robot.getTwister().setPosition(robot.twisterPos4);
        robot.getGrabber().setPosition(robot.grabberOpen);

        robot.getIntake().setPower(.5);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        Thread.sleep(2000);

        robot.getIntake().setPower(0);
        robot.getArmRight().setPosition(robot.armRightTransfer);
        robot.getArmLeft().setPosition(robot.armLeftTransfer);
        robot.getDepositWrist().setPosition(robot.depositWristTransfer2);

        Thread.sleep(2000);

        robot.getGrabber().setPosition(robot.grabberClose2);


        Thread.sleep(1000);

        robot.getIntake().setPower(-.5);
        Thread.sleep(200);
        robot.getIntake().setPower(0);

        robot.getIntakeRoller().setPosition(robot.frontRollerGround);
        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
        robot.getArmLeft().setPosition(robot.armLeftDrop);
        robot.getArmRight().setPosition(robot.armRightDrop);
        Thread.sleep(50);
        robot.getDepositWrist().setPosition(robot.depositWristDrop);
        Thread.sleep(1000);




       }

    @Override
    public void body() throws InterruptedException
    {



    }

    @Override
    public void exit() throws InterruptedException {

    }
}
