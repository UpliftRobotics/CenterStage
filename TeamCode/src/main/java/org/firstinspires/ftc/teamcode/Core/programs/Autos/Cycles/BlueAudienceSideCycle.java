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

        robot.getDepositWrist().setPosition(robot.depositWristStore);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);

        robot.getTwister().setPosition(robot.twisterPos4);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getArmLeft().setPosition(robot.armLeftStore);

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
            //left
            if(location == 0 || location == -1 )
            {

                robot.getBackLeft().setPower(0.5);
                robot.getBackRight().setPower(0.5);
                robot.getFrontLeft().setPower(0.5);
                robot.getFrontRight().setPower(0.5);
                Thread.sleep(800);

                turn(-80, 0.5);

                Thread.sleep(1500);

                robot.getBackLeft().setPower(0.4);
                robot.getBackRight().setPower(0.4);
                robot.getFrontLeft().setPower(0.4);
                robot.getFrontRight().setPower(0.4);

                Thread.sleep(550);

                robot.getBackLeft().setPower(-0.5);
                robot.getBackRight().setPower(-0.5);
                robot.getFrontLeft().setPower(-0.5);
                robot.getFrontRight().setPower(-0.5);

                Thread.sleep(600);
                //drop position
//                driveToPosition(100, 15, 0.7, 0);
//                driveToPosition(94, 28, 0.5, -45, 2);
//
//                driveToPosition(103, 17, 0.5, -45, 2);
//                driveToPosition(103, 50, 0.7, 0);
//                driveToPosition(103, 58, 0.7, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);
//
//                Thread.sleep(1000);
//
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//
//                driveToPosition(110, 59, 0.5, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
//
//                intake(.5);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntake().setPower(.5);
//                Thread.sleep(1000);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
//                robot.getArmLeft().setPosition(robot.armLeftTransfer);
//                robot.getArmRight().setPosition(robot.armRightTransfer);
//                Thread.sleep(500);
//                robot.getGrabber().setPosition(robot.grabberClose2);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(70, 58, 0.7, 85);
//
//                driveToPosition(30, 58, 0.7, 85);
//
//                robot.getIntake().setPower(-0.5);
//                Thread.sleep(200);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(2.5, 25, 0.8, 92);
//
//                robot.getArmLeft().setPosition(robot.armLeftDrop);
//                robot.getArmRight().setPosition(robot.armRightDrop);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
//                Thread.sleep(100);
//                robot.getDepositWrist().setPosition(robot.depositWristDrop);
//
//                Thread.sleep(1000);
//
//                claw("open");
//                Thread.sleep(1000);
//
//                driveToPosition(5, 25, 0.8, 92);

//                reset(true, false);
//                Thread.sleep(500);


            }

            //middle
            if(location == 1 )
            {

                robot.getBackLeft().setPower(0.7);
                robot.getBackRight().setPower(0.7);
                robot.getFrontLeft().setPower(0.7);
                robot.getFrontRight().setPower(0.7);
                Thread.sleep(900);


                robot.getBackLeft().setPower(-0.8);
                robot.getBackRight().setPower(-0.8);
                robot.getFrontLeft().setPower(-0.8);
                robot.getFrontRight().setPower(-0.8);

                Thread.sleep(300);
                //drop position
//                driveToPosition(101, 32, 0.7, 0);
//
//                driveToPosition(102, 20, 0.7, 0);
//                driveToPosition(113, 30, 0.7, 0);
//                driveToPosition(104, 50, 0.7, 0);
//                driveToPosition(104, 58, 0.7, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);
//
//                Thread.sleep(1000);
//
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//
//                driveToPosition(109, 57, 0.5, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
//
//                intake(.5);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntake().setPower(.5);
//                Thread.sleep(1000);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
//                robot.getArmLeft().setPosition(robot.armLeftTransfer);
//                robot.getArmRight().setPosition(robot.armRightTransfer);
//                Thread.sleep(500);
//                robot.getGrabber().setPosition(robot.grabberClose2);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(70, 60, 0.7, 85);
//
//                driveToPosition(30, 58, 0.7, 85);
//
//                robot.getIntake().setPower(-0.5);
//                Thread.sleep(200);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(2.5, 30, 0.8, 92);
//
//                robot.getArmLeft().setPosition(robot.armLeftDrop);
//                robot.getArmRight().setPosition(robot.armRightDrop);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
//                Thread.sleep(100);
//                robot.getDepositWrist().setPosition(robot.depositWristDrop);
//
//                Thread.sleep(1000);
//
//                claw("open");
//                Thread.sleep(1000);
//
//                driveToPosition(5, 30, 0.8, 92);
//
//






            }

            // right

            if(location == 2 )
            {

                robot.getBackLeft().setPower(0.5);
                robot.getBackRight().setPower(0.5);
                robot.getFrontLeft().setPower(0.5);
                robot.getFrontRight().setPower(0.5);
                Thread.sleep(800);

                turn(88, 0.5);

                Thread.sleep(1500);

                robot.getBackLeft().setPower(0.4);
                robot.getBackRight().setPower(0.4);
                robot.getFrontLeft().setPower(0.4);
                robot.getFrontRight().setPower(0.4);

                Thread.sleep(400);

                robot.getBackLeft().setPower(-0.5);
                robot.getBackRight().setPower(-0.5);
                robot.getFrontLeft().setPower(-0.5);
                robot.getFrontRight().setPower(-0.5);

                Thread.sleep(300);
                //drop position
//                driveToPosition(108, 28, 0.7, 0);
//
//                driveToPosition(109, 15, 0.7, 0);
//                driveToPosition(96, 17, 0.8, 0);
//                driveToPosition(96, 50, 0.7, -5);
//
//                driveToPosition(100, 58, 0.7, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);
//
//                Thread.sleep(1000);
//
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//
//                driveToPosition(109, 58, 0.5, 85);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
//
//                intake(.5);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntake().setPower(.5);
//                Thread.sleep(1000);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
//                robot.getArmLeft().setPosition(robot.armLeftTransfer);
//                robot.getArmRight().setPosition(robot.armRightTransfer);
//                Thread.sleep(500);
//                robot.getGrabber().setPosition(robot.grabberClose2);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(70, 58, 0.7, 85);
//
//                driveToPosition(30, 58, 0.7, 85);
//
//                robot.getIntake().setPower(-0.5);
//                Thread.sleep(200);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(2.5, 35, 0.8, 92);
//
//                robot.getArmLeft().setPosition(robot.armLeftDrop);
//                robot.getArmRight().setPosition(robot.armRightDrop);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
//                Thread.sleep(100);
//                robot.getDepositWrist().setPosition(robot.depositWristDrop);
//
//                Thread.sleep(1000);
//
//                claw("open");
//                Thread.sleep(1000);
//
//                driveToPosition(5, 35                                                                                                                                   , 0.8, 92);



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
