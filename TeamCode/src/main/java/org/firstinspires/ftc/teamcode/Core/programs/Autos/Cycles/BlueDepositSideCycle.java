package org.firstinspires.ftc.teamcode.Core.programs.Autos.Cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Blue Deposit Side Cycle", group = "Opmodes")
public class BlueDepositSideCycle extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException
    {
        claw("open");

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

        Thread.sleep(1000);

        robot.webcam.setPipeline(robot.pipelineBlueDepositSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlueDepositSide.location;
        odom.setOdometryPosition(48, 0, 0);

        if(!goPark)
        {
            //left
            if(location == 0 || location == -1 )
            {
                //drop purple pixel position
                driveToPosition(42, 28, 0.5, 0);

                //drop yellow pixel position
                driveToPosition(42, 15, 0.5, 0);
                driveToPosition(5, 24, 0.5, 85);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

                Thread.sleep(1000);

                deposit(200, 0.5);
                Thread.sleep(1000);

                claw("open");
                Thread.sleep(2000);
                reset(true, false);

                Thread.sleep(2000);


            }

            //middle
            if(location == 1 )
            {
                //drop purple pixel position
                driveToPosition(48, 32, 0.5, 0);

                //drop yellow pixel position
                driveToPosition(48, 19, 0.5, 0);
                driveToPosition(5, 30, 0.5, 80);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

                Thread.sleep(1000);

                deposit(200, 0.5);

                Thread.sleep(500);

                claw("open");
                Thread.sleep(2000);
                reset(true, false);

            }

            // right

            if(location == 2 )
            {
                //drop purple pixel position
                driveToPosition(48, 20, 0.5, 0);
                driveToPosition(56, 28, 0.5, 50);


                //drop yellow pixel position
                driveToPosition(42, 14, 0.5, 50);
                driveToPosition(5, 40, 0.5, 80);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

                Thread.sleep(1000);

                deposit(200, 0.5);
                Thread.sleep(500);

                claw("open");
                Thread.sleep(2000);
                reset(true, false);

            }

//            Thread.sleep(1000);
//            driveToPosition(12, 58, 0.5, 80);
//
//            driveToPosition(36, 61, 0.5, 77);
//            driveToPosition(66, 62, 0.5, 75);



//            intake(-0.175);
//
//            reset(false, true);
//
//            cycles("blue", 2);
        }


        //park

//        driveToPosition(10, -10, 0.6, 90);
//        driveToPosition(5, -10, 0.6, 90);




    }

    @Override
    public void exit() throws InterruptedException {

    }
}
