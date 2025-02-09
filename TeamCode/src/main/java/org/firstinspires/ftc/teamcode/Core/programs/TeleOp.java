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

                robot.getExtension().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.getExtension().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.getSlideLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.getSlideRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.getSlideRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                robot.getIntakeRoller().setPosition(robot.frontRollerGround);

                robot.getArmLeft().setPosition(robot.armLeftStore);
                robot.getArmRight().setPosition(robot.armRightStore);
                robot.getDepositWrist().setPosition(robot.depositWristStore);
                robot.getGrabber().setPosition(robot.grabberOpen);
                robot.getTwister().setPosition(robot.twisterPos4);

                robot.getFrontRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.getFrontLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.getBackRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.getBackLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        }

        @Override

        public void bodyLoop() throws InterruptedException
        {

//                telemetry.addData("color: ", robot.getPixelDetectorRight().alpha());
//                telemetry.update();

//                telemetry.addData("slide pos left" , robot.getSlideLeft().getCurrentPosition());
//                telemetry.addData("slide pos right" , robot.getSlideRight().getCurrentPosition());

//                telemetry.addData("left distance", robot.getPixelDetectorLeft().getDistance(DistanceUnit.MM));
//                telemetry.addData("right distance", robot.getPixelDetectorRight().getDistance(DistanceUnit.MM));

//                telemetry.addData("extension pos" , robot.getExtension().getCurrentPosition());
////
////
//                telemetry.update();
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
