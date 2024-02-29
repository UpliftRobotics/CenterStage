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
        robot.getExtension().setPower(0);
    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineRedDepositSide.location;
//        int location = 1;
        odom.setOdometryPosition(48, 144, 180);

        //left
        if(location == 0 || location == -1 )
        {
            //drop purple pixel position
            driveToPosition(48, 126, 0.6, 180);
            driveToPosition(54, 115, 0.6, 130, 2);

//            Thread.sleep(2000);


            //drop yellow pixel position
            driveToPosition(42, 116, 0.9, 130);
            driveToPosition(13.25, 116, 0.6, 100,2);

            robot.frontWebcam.closeCameraDevice();

//            driveToAprilTag(9.75, 116, 100);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);


            deposit(0, 0.5);

            Thread.sleep(2000);
            claw("open");
            Thread.sleep(1000);


            odom.setOdometryPosition(13.25, 116, 100);
            driveToPosition(20, 116, 0.9, 100);

            reset(true, false);

            driveToPosition(20, 144, 0.9, 100);
            driveToPosition(16, 144, 0.9, 100);


            Thread.sleep(1000);


        }

        //middle
        if(location == 1 )
        {
            driveToPosition(48, 113, 0.8, 180);

            //drop yellow pixel position
            driveToPosition(48, 125, 0.8, 180);
            driveToPosition(12.5, 121.5, 0.5, 95,2);

            robot.frontWebcam.closeCameraDevice();

//            driveToAprilTag(9.25, 121.5, 95);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);


            deposit(0, 0.5);

            Thread.sleep(2000);
            claw("open");
            Thread.sleep(1000);

            odom.setOdometryPosition(9.5, 121.5, 95);
            driveToPosition(15, 121.5, 0.9, 100);
            reset(true, false);

            driveToPosition(15, 140, 0.9, 85);


            Thread.sleep(1000);



        }

        // right

        if(location == 2 )
        {
            //drop purple pixel position
            driveToPosition(38, 118, 0.9, 180);

//                drop yellow pixel position
            driveToPosition(38, 130, 0.9, 180);
            driveToPosition(11.5, 127, 0.5, 95,2);

            robot.frontWebcam.closeCameraDevice();

//            driveToAprilTag(9.75, 127, 95);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);


            deposit(0, 0.5);

            Thread.sleep(2000);
            claw("open");
            Thread.sleep(1000);


            odom.setOdometryPosition(11.5, 127, 95);
            driveToPosition(15, 127, 0.9, 95);

            reset(true, false);


            driveToPosition(15, 140, 0.9, 95);


            Thread.sleep(1000);
        }

//        Thread.sleep(1000);
//
//        intake(-0.175);
//        Thread.sleep(5000);
//        intake(0);



        //park
//        driveToPosition(6, 152, 0.6, 90);
//        reset();

    }

    @Override
    public void exit() throws InterruptedException {

    }
}
