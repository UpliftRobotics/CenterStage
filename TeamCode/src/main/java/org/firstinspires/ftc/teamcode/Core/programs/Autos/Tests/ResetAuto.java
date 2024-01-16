package org.firstinspires.ftc.teamcode.Core.programs.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;

@Autonomous(name = "ResetAuto", group = "Opmodes")
public class ResetAuto extends UpliftAutoImpl {

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

robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
robot.getIntakeRoller().setPosition(robot.frontRollerGround);
robot.getGrabber().setPosition(robot.grabberOpen);
robot.getArmLeft().setPosition(robot.armLeftReset);
robot.getArmRight().setPosition(robot.armRightReset);
robot.getDepositWrist().setPosition(robot.depositWristStore);
robot.getTwister().setPosition(robot.twisterPos4);
Thread.sleep(20000);

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
