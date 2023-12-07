package org.firstinspires.ftc.teamcode.Core.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@TeleOp(name = "TestTeleOp", group = "Opmodes")
public class TestTeleOp extends UpliftTele {

        UpliftRobot robot;

        Odometry odom;


        DriveThread driverThread;
        OperatorThread operatorThread;

        @Override
        public void initHardware()
        {
            robot = new UpliftRobot(this);
            odom = robot.odometry;

            createDriveThread(robot);
            createOperatorThread(robot);
        }

        @Override
        public void initAction() {
                driverThread.start();
                operatorThread.start();
                odom.setOdometryPosition(0, 0, 0);


//
//robot.getArmLeft().setPosition(robot.armLeftHold);
//robot.getArmRight().setPosition(robot.armRightHold);
//sleep(20000);


                robot.getDepositWrist().setPosition(robot.depositWristDrop);
                sleep(300);
                robot.getGrabberRight().setPosition(robot.grabberRightOpen);
                robot.getGrabberLeft().setPosition(robot.grabberLeftOpen);
                robot.getArmRight().setPosition(robot.armRightPast);
                robot.getArmLeft().setPosition(robot.armLeftPast);
                sleep(500);
                robot.getDepositWrist().setPosition(robot.depositWristGrab);
                sleep(500);
                robot.getArmRight().setPosition(robot.armRightGrab);
                robot.getArmLeft().setPosition(robot.armLeftGrab);

                robot.getSlideRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.getSlideLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.getSlideRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.getPlane().setPosition(1);

        }

        @Override

        public void bodyLoop() throws InterruptedException
        {

                //telemetry.addData("touch sensor" , robot.getExtensionTouch().getValue());
                telemetry.addData("slide left ", robot.getSlideLeft().getCurrentPosition());
                telemetry.addData("slide right ", robot.getSlideRight().getCurrentPosition());
                telemetry.addData("slide pos ",                 (robot.getSlideLeft().getCurrentPosition() + robot.getSlideRight().getCurrentPosition()) /2
                );
//                telemetry.addData("right stick y", robot.opMode.gamepad2.right_stick_y);
//                telemetry.addData("right trigger", robot.opMode.gamepad1.right_trigger);
//                telemetry.addData("left trigger", robot.opMode.gamepad1.left_trigger);
//                telemetry.addData("extension", robot.getExtension().getCurrentPosition());


                telemetry.update();
        }

        @Override
        public void exit()
        {
                driverThread.end();
                operatorThread.end();
        }
        public void createDriveThread(UpliftRobot robot1)
        {

                driverThread = new DriveThread(robot1);
                telemetry.addData("Driver Thread started", driverThread.toString());

        }

        public void createOperatorThread(UpliftRobot robot1) {

                operatorThread = new OperatorThread(robot1);
                telemetry.addData("Operator Thread started", operatorThread.toString());

        }

}
