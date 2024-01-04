package org.firstinspires.ftc.teamcode.Core.programs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread2;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestTeleOp", group = "Opmodes")
public class TeleOp extends UpliftTele {

        UpliftRobot robot;

        Odometry odom;


        DriveThread driveThread;

        DriveThread2 driveThread2;
        OperatorThread operatorThread;



        @Override
        public void initHardware()
        {
            robot = new UpliftRobot(this);
            odom = robot.odometry;

            createDriveThread(robot);
            createDriveThread2(robot);
            createOperatorThread(robot);
        }

        @Override
        public void initAction() {
                driveThread.start();
                driveThread2.start();
                operatorThread.start();

                odom.setOdometryPosition(0, 0, 0);


//
//robot.getArmLeft().setPosition(robot.armLeftHold);
//robot.getArmRight().setPosition(robot.armRightHold);
//sleep(20000);


//                robot.getDepositWrist().setPosition(robot.depositWristDrop);
//                sleep(300);
//
//                robot.getGrabber().setPosition(robot.grabberOpenPos);
//
//                robot.getArmRight().setPosition(robot.armRightPast);
//                robot.getArmLeft().setPosition(robot.armLeftPast);
//                sleep(500);
//
//                robot.getDepositWrist().setPosition(robot.depositWristGrab);
//                sleep(500);
//
//                robot.getArmRight().setPosition(robot.armRightGrab);
//                robot.getArmLeft().setPosition(robot.armLeftGrab);
//
//                robot.getSlideRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.getSlideLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                robot.getSlideRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                robot.getPlane().setPosition(1);

        }

        @Override

        public void bodyLoop() throws InterruptedException
        {
                telemetry.addData("color: ", robot.getPixelDetectorRight().alpha());
                telemetry.update();
        }

        @Override
        public void exit()
        {
                driveThread.end();
                driveThread2.end();
                operatorThread.end();
        }
        public void createDriveThread(UpliftRobot robot1)
        {

                driveThread = new DriveThread(robot1);
                telemetry.addData("Driver Thread started", driveThread.toString());

        }

        public void createDriveThread2(UpliftRobot robot1)
        {

                driveThread2 = new DriveThread2(robot1);
                telemetry.addData("Driver Thread started", driveThread2.toString());

        }

        public void createOperatorThread(UpliftRobot robot1) {

                operatorThread = new OperatorThread(robot1);
                telemetry.addData("Operator Thread started", operatorThread.toString());

        }

}
