package org.firstinspires.ftc.teamcode.Core.programs.Autos.Cycles;

import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.blueLeftStack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;


@Autonomous(name = "Blue Audience Side Cycle", group = "Opmodes")
public class BlueAudienceSideCycle extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException {

        claw("open");

        robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);

        robot.getTwister().setPosition(robot.twisterPos4);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getArmLeft().setPosition(robot.armLeftStore);

        Thread.sleep(1000);

//        claw("close1");

        robot.webcam.setPipeline(robot.pipelineBlueDepositSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlueAudienceSide.location;
        odom.setOdometryPosition(100, 0, 0);

        if(!goPark)
        {
            //left
            if(location == 0 || location == -1 )
            {
//                //drop position
//                driveToPosition(106, 28, 0.5, 0);
//                driveToPosition(3.5, 10, 0.7, 90, 2);
//                Thread.sleep(1000);
//
//
//
//                deposit(400, 0.1);
//                Thread.sleep(500);
//
//
//
//                claw("open");
//                Thread.sleep(1000);
//                reset(true, false);
//
//
//                //outtake position
//                driveToPosition(10, 15, 0.5, 93);
//
//                //extend to outtake
//                extensionPID(400, 200, 0.5);

            }

            //middle
            if(location == 1 )
            {
//                //drop position
//                driveToPosition(30, 6, 0.8, 180);
//                driveToPosition(3.5, 16, 0.7, 90, 2);
//                Thread.sleep(1000);
//
//
//                deposit(400, 0.1);
//                Thread.sleep(500);
//
//
//                claw("open");
//                Thread.sleep(1000);
//                reset(true, false);
//
//
//                //outtake position
//                driveToPosition(10, 20, 0.5, 93);
//
//                //extend to outtake
//                extensionPID(500, 250, 0.5);
            }

            // right

            if(location == 2 )
            {
                //drop position
                driveToPosition(108, 28, 0.7, 0);

                driveToPosition(109, 15, 0.7, 0);
                driveToPosition(96, 17, 0.8, 0);
                driveToPosition(96, 50, 0.7, -5);

                driveToPosition(100, 58, 0.7, 85);

                Thread.sleep(1000);

                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);

                Thread.sleep(1000);

                driveToPosition(109, 58, 0.5, 85);

                Thread.sleep(1000);

                robot.getIntakeRoller().setPosition(robot.frontRollerGround);

                intake(.5);

                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
                robot.getIntake().setPower(.5);
                Thread.sleep(1000);
                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
                robot.getArmLeft().setPosition(robot.armLeftTransfer);
                robot.getArmRight().setPosition(robot.armRightTransfer);
                Thread.sleep(500);
                robot.getGrabber().setPosition(robot.grabberClose2);
                robot.getIntake().setPower(0);

                driveToPosition(70, 58, 0.5, 85);

//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);

                driveToPosition(30, 58, 0.5, 85);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);

                driveToPosition(20, 30, 0.5, 85);
                





//                claw("open");
//                Thread.sleep(1000);
//                reset(true, false);
//
//                //outtake position
//                driveToPosition(10, 15, 0.5, 93);
//
//                //extend to outtake
//                extensionPID(600, 300, 0.5);

            }

//            Thread.sleep(100);
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
