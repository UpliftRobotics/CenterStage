package org.firstinspires.ftc.teamcode.Core.programs.Autos.Cycles;

import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.blueLeftStack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;


@Autonomous(name = "Blue Audience Cycle", group = "Opmodes")
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

        robot.getDepositWrist().setPosition(robot.depositWristStore);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);

        robot.getTwister().setPosition(robot.twisterPos4);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getArmLeft().setPosition(robot.armLeftStore);

        robot.getIntake().setPower(0.1);
        Thread.sleep(1000);

//        claw("close1");-

        robot.webcam.setPipeline(robot.pipelineBlueDepositSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlueAudienceSide.location;
        odom.setOdometryPosition(100, 0, 0);

        if(!goPark)
        {

            robot.getExtension().setPower(-.1);
            //left
            if(location == 0 || location == -1 )
            {

//                drop position
                driveToPosition(100, 15, 0.7, 0);
                driveToPosition(96, 31, 0.5, -45, 2);

                driveToPosition(104, 16, 0.5, -45, 2);
                driveToPosition(104, 50, 0.7, 0);
                driveToPosition(105, 58, 0.7, 90);

                Thread.sleep(1000);

                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);

                Thread.sleep(1000);

                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);

                driveToPosition(114, 59, 0.5, 90);

                Thread.sleep(1000);

                robot.getIntakeRoller().setPosition(robot.frontRollerGround);

                intake(.7);

                driveToPosition(75, 60, 0.9, 90);

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

                driveToPosition(75, 60, 0.9, 90);

                driveToPosition(30, 57, 0.9, 90);

                robot.getIntake().setPower(-0.5);
                Thread.sleep(200);
                robot.getIntake().setPower(0);

                driveToPosition(15, 40, 0.9, 92);
                driveToPosition(5, 28, 0.9, 92);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

                Thread.sleep(2000);

                deposit(200, 0.5);
                robot.getTwister().setPosition(0.8);

                Thread.sleep(1000);

                claw("open");
                Thread.sleep(1000);

                driveToPosition(10, 28, 0.8, 90);
                robot.getTwister().setPosition(robot.twisterPos4);

                reset(true, false);

                Thread.sleep(1000);


            }

            //middle
            if(location == 1 )
            {


//                drop position
                driveToPosition(101, 32, 0.8, 0);

                driveToPosition(102, 20, 0.9, 0);
                driveToPosition(114, 30, 0.9, 0);
                driveToPosition(113, 50, 0.9, 0);
                driveToPosition(113, 58, 0.9, 90);

                Thread.sleep(1000);

                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);

                Thread.sleep(1000);

                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);

                driveToPosition(116, 57, 0.5, 92);

                Thread.sleep(1000);

                robot.getIntakeRoller().setPosition(robot.frontRollerGround);

                intake(.7);

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

                driveToPosition(75, 62, 0.9, 90);

                driveToPosition(25, 60, 0.9, 90);

                robot.getIntake().setPower(-0.5);
                Thread.sleep(200);
                robot.getIntake().setPower(0);

                driveToPosition(15, 56, 0.8, 92);
                driveToPosition(6, 45, 0.8, 92);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

                Thread.sleep(2000);

                deposit(200, 0.5);

                Thread.sleep(1000);

                claw("open");
                Thread.sleep(1000);

                driveToPosition(10, 45, 0.8, 90);

                reset(true, false);

                Thread.sleep(1000);

            }

            // right

            if(location == 2 )
            {
                //drop position
                driveToPosition(110, 28, 0.7, 0);

                driveToPosition(109, 15, 0.7, 0);
                driveToPosition(96, 15, 1, 0);
                driveToPosition(100, 50, 0.85, 0);

                driveToPosition(100, 57, 0.85, 88);

                Thread.sleep(1000);

                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);

                Thread.sleep(1000);

                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);

                driveToPosition(113.5, 57, 0.5, 90);

                Thread.sleep(1000);

                robot.getIntakeRoller().setPosition(robot.frontRollerGround);

                intake(.7);

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

                driveToPosition(75, 58, 0.7, 85);

                driveToPosition(30, 58, 0.7, 85);

                robot.getIntake().setPower(-0.5);
                Thread.sleep(200);
                robot.getIntake().setPower(0);


                driveToPosition(15, 47, 1, 92);
                driveToPosition(5, 42, 1, 92);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);

                Thread.sleep(2000);

                deposit(200, 0.5);
                robot.getTwister().setPosition(0.9);

                Thread.sleep(1000);

                claw("open");
                Thread.sleep(1000);

                driveToPosition(10, 42, 0.8, 90);
                robot.getTwister().setPosition(robot.twisterPos4);

                reset(true, false);

                Thread.sleep(1000);








                //2nd cycle

//                driveToPosition(30, 50, 0.7, 85);
//                driveToPosition(100, 50, 0.7, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
//
//                Thread.sleep(1000);
//
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//
//                driveToPosition(110, 58, 0.5, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
//
//                intake(.5);










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
