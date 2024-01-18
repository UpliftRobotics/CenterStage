package org.firstinspires.ftc.teamcode.Core.programs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
//import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread2;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Opmodes")
public class TeleOp extends UpliftTele {

        UpliftRobot robot;

        Odometry odom;


        DriveThread driveThread;

//        DriveThread2 driveThread2;
        OperatorThread operatorThread;



        @Override
        public void initHardware()
        {
            robot = new UpliftRobot(this);
            odom = robot.odometry;

            createDriveThread(robot);
//            createDriveThread2(robot);
            createOperatorThread(robot);
        }

        @Override
        public void initAction() throws InterruptedException {
                driveThread.start();
//                driveThread2.start();
                operatorThread.start();

                odom.setOdometryPosition(0, 0, 0);

//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
//                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
////
//                robot.getArmRight().setPosition(robot.armRightStore);
//                robot.getArmLeft().setPosition(robot.armLeftStore);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//                robot.getTwister().setPosition(robot.twisterPos4);
//                robot.getGrabber().setPosition(robot.grabberOpen);
//
//
//                Thread.sleep(1000);
////
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//
//                Thread.sleep(2000);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
//                robot.getArmLeft().setPosition(robot.armLeftTransfer);
//                robot.getArmRight().setPosition(robot.armRightTransfer);
//                Thread.sleep(4000);
//                robot.getGrabber().setPosition(robot.grabberClose2);
//                Thread.sleep(4000);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
//                robot.getArmRight().setPosition(robot.armRightDrop);
//                robot.getArmLeft().setPosition(robot.armLeftDrop);
//                Thread.sleep(1000);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
                robot.getIntakeRoller().setPosition(robot.frontRollerGround);

                robot.getArmLeft().setPosition(robot.armLeftStore);
                robot.getArmRight().setPosition(robot.armRightStore);
                robot.getDepositWrist().setPosition(robot.depositWristStore);
                robot.getGrabber().setPosition(robot.grabberOpen);
                robot.getTwister().setPosition(robot.twisterPos4);


        }

        @Override

        public void bodyLoop() throws InterruptedException
        {
//                telemetry.addData("color: ", robot.getPixelDetectorRight().alpha());
//                telemetry.update();

//                telemetry.addData("slide pos left" , robot.getSlideLeft().getCurrentPosition());
//                telemetry.addData("slide pos right" , robot.getSlideRight().getCurrentPosition());

                telemetry.addData("left distance", robot.getPixelDetectorLeft().getDistance(DistanceUnit.MM));
                telemetry.addData("right distance", robot.getPixelDetectorRight().getDistance(DistanceUnit.MM));


                telemetry.update();
        }

        @Override
        public void exit()
        {
                driveThread.end();
//                driveThread2.end();
                operatorThread.end();
        }
        public void createDriveThread(UpliftRobot robot1)
        {

                driveThread = new DriveThread(robot1);
                telemetry.addData("Driver Thread started", driveThread.toString());

        }

//        public void createDriveThread2(UpliftRobot robot1)
//        {
//
//                driveThread2 = new DriveThread2(robot1);
//                telemetry.addData("Driver Thread started", driveThread2.toString());
//
//        }

        public void createOperatorThread(UpliftRobot robot1) {

                operatorThread = new OperatorThread(robot1);
                telemetry.addData("Operator Thread started", operatorThread.toString());

        }

}
