package org.firstinspires.ftc.teamcode.Core.programs.Autos.NoCycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Red Deposit 2 + 0", group = "Opmodes")
public class RedDeposit2Plus0 extends UpliftAutoImpl
{
    Odometry odom;

    public void initHardware()
    {
        robot = new UpliftRobot(this);
        odom = robot.odometry;
    }

    @Override
    public void initAction() throws InterruptedException {

        robot.getExtension().setPower(.05);
        claw("open");

        robot.getIntake().setPower(.2);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);


        robot.getTwister().setPosition(robot.twisterPos4);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getArmLeft().setPosition(robot.armLeftStore);
        robot.getDepositWrist().setPosition(robot.depositWristTransfer1);

        Thread.sleep(3000);

        robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
        robot.getArmRight().setPosition(robot.armRightTransfer);
        robot.getArmLeft().setPosition(robot.armLeftTransfer);

        Thread.sleep(500);

        claw("close1");
        robot.getIntake().setPower(0);

        Thread.sleep(1000);

        robot.frontWebcam.setPipeline(robot.pipelineRedDepositSide);
    }

    @Override
    public void body() throws InterruptedException
    {
//        int location = robot.pipelineRedDepositSide.location;
        int location = 1;
        odom.setOdometryPosition(48, 144, 0);

        //left
        if(location == 0 || location == -1 )
        {
            //drop position


        }

        //middle
        if(location == 1 )
        {
            //drop position
            driveToPosition(30, 135, 0.8, 0);
            driveToPosition(14, 125, 0.7, 90);

            robot.frontWebcam.closeCameraDevice();

            driveToAprilTag(14.8, 125, 90);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);


            deposit(0, 0.5);
            Thread.sleep(500);
            claw("open");
            reset(true, false);

            odom.setOdometryPosition(14, 125, 90);
            driveToPosition(15, 140, 0.9, 90);
            Thread.sleep(1000);
        }

        // right

        if(location == 2 )
        {
            //drop position
            driveToPosition(30, 130, 0.8, 0);
            driveToPosition(4.5, 135, 0.7, 90);
            Thread.sleep(1000);

            deposit(400, 0.1);
            Thread.sleep(500);

            claw("open");
            Thread.sleep(1000);
            reset(true, false);


            //outtake position
            driveToPosition(24, 118, 0.5, 90);

        }

        Thread.sleep(1000);

        intake(-0.175);
        Thread.sleep(5000);
        intake(0);



        //park
        driveToPosition(6, 152, 0.6, 90);
//        reset();

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
