package org.firstinspires.ftc.teamcode.Core.Threads;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;

public class OperatorThread extends Thread {


    private UpliftRobot robot;

    private static final String OPERATOR_NAME = "OperatorThreadName";

    private boolean shutDown = false;


    public OperatorThread(UpliftRobot robot) {
        this.robot = robot;

    }

    @Override
    public void run() {
        while (!shutDown) {
            try {
                deposit();
                twister();
                intakeControl();
                automaticPixel();
                frontRollerToggle();
                intakeStore();
                intakeStack();
                drop();
                automaticStore();
                driverOneCycle();
                verticalTwister();

                robot.opMode.telemetry.addData("right slide ticks", robot.getSlideRight().getCurrentPosition());
                robot.opMode.telemetry.addData("left slide ticks", robot.getSlideLeft().getCurrentPosition());
                robot.opMode.telemetry.update();





                // todo: validate user responsiveness and set sleep
//                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();

            }


        }
    }

    public void end() {
        shutDown = true;

        robot.opMode.telemetry.addData("Operator Thread stopped ", shutDown);

        robot.opMode.telemetry.update();
    }

    @Override
    public String toString() {
        return "OperatorThread{" +
                "name=" + OPERATOR_NAME +
                '}';
    }

//    public void intake() {
//        robot.getIntake().setPower(-.6 * robot.opMode.gamepad2.left_stick_y);
//    }






    public void driverOneCycle() throws InterruptedException {
        if (robot.opMode.gamepad1.dpad_down && robot.oneDriver)
        {
            if (robot.depositStage == 0 || robot.depositStage == -1)// intake on the ground for pick up, move inside the robot
            {
                robot.extensionPower = -.6;
                robot.getArmRight().setPosition(robot.armRightStore);
                robot.getArmLeft().setPosition(robot.armLeftStore);
                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
                robot.getTwister().setPosition(robot.twisterPos4);
                robot.getGrabber().setPosition(robot.grabberOpen);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
                robot.intakePower = .5;
                Thread.sleep(1000);
                robot.intakePower = 0;
                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
                robot.getArmLeft().setPosition(robot.armLeftTransfer);
                robot.getArmRight().setPosition(robot.armRightTransfer);
                Thread.sleep(500);
                robot.getGrabber().setPosition(robot.grabberClose2);
                Thread.sleep(200);
                robot.intakePower = -.3;
                Thread.sleep(150);
                robot.intakePower = 0;
                robot.extensionPower = 0;
                robot.depositStage = 1;
            }
            else if (robot.depositStage == 1) //intake is in the robot, transfer by grabbing the pixels and then sending the intake out
            {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                robot.getArmRight().setPosition(robot.armRightDrop);
                robot.getArmLeft().setPosition(robot.armLeftDrop);
                Thread.sleep(200);
//                robot.slidePower = 1;
                robot.getDepositWrist().setPosition(robot.depositWristDrop);
                Thread.sleep(400);
//                robot.slidePower = 0;
                robot.depositStage++;

            }
        }

    }

    public void deposit() throws InterruptedException {
        if (robot.opMode.gamepad2.y)
            {
                if (robot.depositStage == 0 || robot.depositStage == -1)// intake on the ground for pick up, move inside the robot
                {
                    robot.extensionPower = -.6;
                    robot.getArmRight().setPosition(robot.armRightStore);
                robot.getArmLeft().setPosition(robot.armLeftStore);
                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
                robot.getTwister().setPosition(robot.twisterPos4);
                robot.getGrabber().setPosition(robot.grabberOpen);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
                robot.intakePower = .5;
                Thread.sleep(1250);
                robot.intakePower = 0;
                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
                robot.getArmLeft().setPosition(robot.armLeftTransfer);
                robot.getArmRight().setPosition(robot.armRightTransfer);
                Thread.sleep(500);
                robot.getGrabber().setPosition(robot.grabberClose2);
                Thread.sleep(200);
                robot.intakePower = -.3;
                Thread.sleep(150);
                robot.intakePower = 0;
                robot.extensionPower = 0;
                robot.getExtension().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.getExtension().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.depositStage = 1;

                }
                else if (robot.depositStage == 1) //intake is in the robot, transfer by grabbing the pixels and then sending the intake out
                {
                    robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                robot.getArmRight().setPosition(robot.armRightDrop);
                robot.getArmLeft().setPosition(robot.armLeftDrop);
                Thread.sleep(50);
                robot.getDepositWrist().setPosition(robot.depositWristDrop);
                    robot.getIntakeRoller().setPosition(robot.frontRollerGround);
                    Thread.sleep(200);
                robot.depositStage++;

                }
            }

        }



    public void twister() throws InterruptedException{
        if(robot.opMode.gamepad2.left_bumper && robot.depositStage == 2)
        {
            robot.getTwister().setPosition(robot.getTwister().getPosition() - .1);
            Thread.sleep(120);

        }

        if (robot.opMode.gamepad2.right_bumper && robot.depositStage == 2)
        {
            robot.getTwister().setPosition(robot.getTwister().getPosition() + .1);
            Thread.sleep(120);

        }
    }

    public void intakeControl() throws InterruptedException {
        if (robot.opMode.gamepad1.left_bumper)
        {
            if (robot.intakeHeight == 1)
            {
                    robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftReset);
                    robot.getIntakeArmRight().setPosition(robot.intakeArmRightReset);
                    robot.getIntakeRoller().setPosition(robot.frontRollerGround);
                    if (robot.oneDriver)
                    {
                        robot.intakePower = 1;
                    }
                    Thread.sleep(200);
                    robot.intakeHeight = 0;


            }
            else if ( robot.intakeHeight == 0)
            {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
                if (robot.oneDriver)
                {
                    robot.intakePower = 0;

                }
                Thread.sleep(200);
                robot.intakeHeight = 1;
            }


        }
    }

    public void intakeStore()
    {
        if(robot.opMode.gamepad2.a)
        {
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
        }
    }

    public void frontRollerToggle()
    {
        if(robot.opMode.gamepad2.x)
        {
                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
        }
    }

    public void intakeStack()
    {
        if(robot.opMode.gamepad2.dpad_up)
        {
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
            robot.getIntakeRoller().setPosition(robot.frontRollerStack);
        }
        if(robot.opMode.gamepad2.dpad_left)
        {
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
            robot.getIntakeRoller().setPosition(robot.frontRollerStack);

        }
        if(robot.opMode.gamepad2.dpad_down)
        {
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
            robot.getIntakeRoller().setPosition(robot.frontRollerStack);

        }
        if(robot.opMode.gamepad2.dpad_right)
        {
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
            robot.getIntakeRoller().setPosition(robot.frontRollerStack);

        }
    }

    public void automaticPixel() throws Exception
    {
        if (robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 2
                && robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 2
                && robot.depositStage == 0)
        {
            Thread.sleep(200);
            if (robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 2
                    && robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 2
                    && robot.depositStage == 0) {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
                robot.depositStage = -1;
                robot.opMode.gamepad1.rumbleBlips(2);
                robot.intakePower = 0;
//                robot.extensionPower = -.2;
            }
        }
    }

    public void automaticStore()
    {
        if( robot.getSlideRight().getCurrentPosition() < -2200 || robot.getSlideLeft().getCurrentPosition() < -2200)
        {
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);
            robot.getIntakeRoller().setPosition(robot.frontRollerStore);
            robot.extensionPower = -0.2;
        }
    }

    public void drop () throws Exception
    {
        if (((robot.opMode.gamepad2.right_trigger > .25 || robot.opMode.gamepad2.left_trigger > .25) && robot.depositStage == 2)
                || ((robot.opMode.gamepad1.dpad_down) && robot.oneDriver && robot.depositStage == 2))
        {
            robot.getGrabber().setPosition(robot.grabberOpen);
            Thread.sleep(400);
//            if(robot.oneDriver)
//            {
//                robot.slidePower = -1;
//            }
            robot.getArmRight().setPosition(robot.armRightStore);
            robot.getArmLeft().setPosition(robot.armLeftStore);
            robot.getDepositWrist().setPosition(robot.depositWristStore);
            robot.getTwister().setPosition(robot.twisterPos4);
            robot.getIntakeRoller().setPosition(robot.frontRollerGround);
            Thread.sleep(1000);
//            if(robot.oneDriver)
//            {
//                robot.slidePower = 0;
//            }
            robot.depositStage = 0;
        }
    }

    public void verticalTwister() throws Exception
    {
        if (robot.opMode.gamepad2.b && robot.depositStage == 2 )
        {
            if (robot.twisterVerticalStage == 0)
            {
                robot.getTwister().setPosition(robot.twisterPos1);
                robot.twisterVerticalStage++;
                Thread.sleep(200);
            }
            else
            {
                robot.getTwister().setPosition(robot.twisterPos7);
                robot.twisterVerticalStage--;
                Thread.sleep(200);
            }
        }
    }

}