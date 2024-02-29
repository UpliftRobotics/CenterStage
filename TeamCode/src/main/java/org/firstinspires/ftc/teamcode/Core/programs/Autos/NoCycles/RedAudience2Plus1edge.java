package org.firstinspires.ftc.teamcode.Core.programs.Autos.NoCycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@Autonomous(name = "Red Audience 2 + 1 Edge", group = "Opmodes")
public class RedAudience2Plus1edge extends UpliftAutoImpl
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

        robot.frontWebcam.setPipeline(robot.pipelineRedAudienceSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineRedAudienceSide.location;
        odom.setOdometryPosition(100, 144, 180);

        robot.getExtension().setPower(0.1);

        //left
        if(location == 0 || location == -1 )
        {

            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntake().setPower(.5);
            Thread.sleep(1000);
            robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
            robot.getArmLeft().setPosition(robot.armLeftTransfer);
            robot.getArmRight().setPosition(robot.armRightTransfer);
            Thread.sleep(500);

            claw("close1");
            robot.getIntake().setPower(0);
            //drop position
            driveToPosition(107, 119, 0.8, 180);

            driveToPosition(106, 129, 0.8, 180);

            driveToPosition(106, 132, 0.5, 85,2);

            driveToPosition(30, 132, 0.5, 85);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);

            driveToPosition(13, 105, 0.5, 110);


            driveToPosition(-1,105,.3,110);

            deposit(0,0);

            Thread.sleep(500);


            claw("open");

            Thread.sleep(50);

            reset(false,false);

            Thread.sleep(500);

            driveToPosition(8,105,.5,95);

            driveToPosition(8,132,.5,95);



        }

        //middle
        if(location == 1 )
        {

            //drop position
            driveToPosition(100, 113, 0.8, 190);
            driveToPosition(112, 122, 0.8, 182);

            driveToPosition(106, 116, 0.5, 101,2);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
            robot.getIntakeRoller().setPosition(robot.frontRollerStack);


            driveToPosition(111, 116, 0.5, 101,2);



            Thread.sleep(10000);






        }

        // right

        if(location == 2) {
            //drop position
            driveToPosition(100, 125, 0.8, 180);

            driveToPosition(90, 115, 0.7, 215);
            driveToPosition(110, 125, 0.7, 215);

            driveToPosition(106, 85, 0.9, 95, 2);
            Thread.sleep(1000);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);

            Thread.sleep(1000);


            robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
            driveToPosition(108, 82, 0.5, 92);

            Thread.sleep(1000);

            robot.getIntakeRoller().setPosition(robot.frontRollerGround);

            intake(.7);

        }
    }

    @Override
    public void exit() throws InterruptedException {

    }
}
