package org.firstinspires.ftc.teamcode.Core.programs.Autos.NoCycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Red Audience 2 + 1", group = "Opmodes")
public class RedAudience2Plus1 extends UpliftAutoImpl
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

        robot.getIntake().setPower(0.1);
        Thread.sleep(1000);

//        claw("close1");

        robot.webcam.setPipeline(robot.pipelineRedAudienceSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineRedAudienceSide.location;
        odom.setOdometryPosition(100, 144, 180);

        robot.getExtension().setPower(-.2);

        //left
        if(location == 0 || location == -1 ) {
            //drop position
            driveToPosition(108, 119, 0.8, 180);

            driveToPosition(107, 129, 0.8, 180);
            driveToPosition(94, 120, 0.9, 180);
            driveToPosition(94, 90, 0.9, 180);
            driveToPosition(100, 86, 0.9, 90, 2);

            Thread.sleep(1000);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);

            Thread.sleep(1000);

            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            driveToPosition(105, 86, 0.5, 92);

            //Thread.sleep(1000);

          //  robot.getIntakeRoller().setPosition(robot.frontRollerGround);

            //intake(.7);

            driveToPosition(70, 85, .9,90); // under bridge

            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntake().setPower(.5);
            Thread.sleep(1000);
            robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
            robot.getArmLeft().setPosition(robot.armLeftTransfer);
            robot.getArmRight().setPosition(robot.armRightTransfer);
            Thread.sleep(500);

            claw("close2");

            robot.getIntake().setPower(0);

            driveToPosition(25, 95, 0.9,90);

            driveToPosition(-2, 110, 0.9, 92);

            robot.getIntake().setPower(-.5);
            Thread.sleep(400);
            robot.getIntake().setPower(0);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

            Thread.sleep(1000);

            deposit(100, 0.5);
            robot.getTwister().setPosition(0.2);

            Thread.sleep(1000);

            claw("open");
            Thread.sleep(1000);

            driveToPosition(5, 110, 0.8, 92);
            robot.getTwister().setPosition(robot.twisterPos4);

            reset(true, false);

            Thread.sleep(1000);


        }

        //middle
        if(location == 1 )
        {

            //drop position
            driveToPosition(100, 113, 0.8, 190);
            driveToPosition(112, 122, 0.8, 182);

            driveToPosition(105, 97, 0.9, 180);
            driveToPosition(107, 88,  0.9, 90, 2);


            Thread.sleep(1000);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);

            Thread.sleep(1000);


            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            driveToPosition(112, 85, 0.5, 92);

          //  Thread.sleep(1000);

           // robot.getIntakeRoller().setPosition(robot.frontRollerGround);

           // intake(.7);

            driveToPosition(70, 85, .9,90); // under bridge

            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntake().setPower(.5);
            Thread.sleep(1000);
            robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
            robot.getArmLeft().setPosition(robot.armLeftTransfer);
            robot.getArmRight().setPosition(robot.armRightTransfer);
            Thread.sleep(500);

            claw("close2");

            robot.getIntake().setPower(0);

            driveToPosition(30, 95, 0.9,90);

            driveToPosition(0, 115, 0.9, 80);

            robot.getIntake().setPower(-.5);
            Thread.sleep(400);
            robot.getIntake().setPower(0);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

            Thread.sleep(1000);

            deposit(100, 0.5);
            robot.getTwister().setPosition(.2);

            Thread.sleep(1000);

            claw("open");
            Thread.sleep(1000);

            driveToPosition(5, 115, 0.8, 80);
            robot.getTwister().setPosition(robot.twisterPos4);

            reset(true, false);

            Thread.sleep(1000);


        }

        // right

        if(location == 2)
        {
            //drop position
            driveToPosition(100, 125, 0.8, 180);

            driveToPosition(90, 115, 0.7, 215);
            driveToPosition(110, 125, 0.7, 215);

            driveToPosition(106, 85, 0.9, 95,2);
            Thread.sleep(1000);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);

            Thread.sleep(1000);


            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            driveToPosition(108, 82, 0.5, 92);

          //  Thread.sleep(1000);

          //  robot.getIntakeRoller().setPosition(robot.frontRollerGround);

           // intake(.7);

            driveToPosition(70, 90, 0.9,90); // under bridge


            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntake().setPower(.5);
            Thread.sleep(1000);
            robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
            robot.getArmLeft().setPosition(robot.armLeftTransfer);
            robot.getArmRight().setPosition(robot.armRightTransfer);
            Thread.sleep(500);

            claw("close2");

            robot.getIntake().setPower(0);

            driveToPosition(35, 95, 0.9,90);

            driveToPosition(5, 128, 1, 80);
            Thread.sleep(1000);
            driveToPosition(0, 128, 1, 80);

            robot.getIntake().setPower(-.5);
            Thread.sleep(400);
            robot.getIntake().setPower(0);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

            Thread.sleep(1000);

            deposit(100, 0.5);
            robot.getTwister().setPosition(0.2);

            Thread.sleep(1000);

            claw("open");
            Thread.sleep(1000);

            driveToPosition(5, 128, 0.8, 80);
            robot.getTwister().setPosition(robot.twisterPos4);

            reset(true, false);

            Thread.sleep(1000);


        }
    }

    @Override
    public void exit() throws InterruptedException {

    }
}
