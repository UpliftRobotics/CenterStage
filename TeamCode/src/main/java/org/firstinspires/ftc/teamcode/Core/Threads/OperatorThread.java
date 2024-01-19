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
//                intake();
                slides();
                deposit();
                rightTwister();
                leftTwister();
                intakeControl();
                automaticPixel();
                frontRollerToggle();
                intakeStore();
                intakeStack();
                drop();





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

    public void intake() {
        robot.getIntake().setPower(-.6 * robot.opMode.gamepad2.left_stick_y);
    }

    public void slides() {
        double power = .8 * robot.opMode.gamepad2.right_stick_y;
        if (robot.opMode.gamepad2.right_bumper)
            power  = power * .05;

        // if going up stop from overextending
        if (power < 0.0) {
            if (robot.getSlideRight().getCurrentPosition()  < -2800 || robot.getSlideLeft().getCurrentPosition() < -2800) {
                robot.getSlideLeft().setPower(0);
                robot.getSlideRight().setPower(0);
            } else {
                robot.getSlideLeft().setPower(power);
                robot.getSlideRight().setPower(power);
            }
        }
        // stop from overretracting
        else {
            if (robot.getSlideRight().getCurrentPosition() > -10 || robot.getSlideLeft().getCurrentPosition() > -10) {
                robot.getSlideLeft().setPower(0);
                robot.getSlideRight().setPower(0);
            } else {
                robot.getSlideLeft().setPower(power);
                robot.getSlideRight().setPower(power);
            }
        }


        //robot.getSlideRight().setPower(.5 * robot.opMode.gamepad2.right_stick_y);
        //robot.getSlideLeft().setPower(.5 * robot.opMode.gamepad2.right_stick_y);

    }

    public void d1reset() throws InterruptedException {

        if (robot.opMode.gamepad1.left_bumper && robot.depositStage == 0)
        {
            if (robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 3 || robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 3)
            {
                // pull slides in
                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
                robot.getArmLeft().setPosition(robot.armLeftStore);
                robot.getArmRight().setPosition(robot.armRightStore);
                robot.depositStage++;
                Thread.sleep(1000);
            }
            else
            {
                // just pull slides in
            }

        }
    }





    public void deposit() throws InterruptedException {
        if (robot.opMode.gamepad2.y)
            {
                if (robot.depositStage == 0 || robot.depositStage == -1)// intake on the ground for pick up, move inside the robot
                {
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
                robot.depositStage = 1;
                }
                else if (robot.depositStage == 1) //intake is in the robot, transfer by grabbing the pixels and then sending the intake out
                {
                    robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
                robot.getArmRight().setPosition(robot.armRightDrop);
                robot.getArmLeft().setPosition(robot.armLeftDrop);
                Thread.sleep(200);
                robot.getDepositWrist().setPosition(robot.depositWristDrop);
                Thread.sleep(200);
                robot.depositStage++;

                }
            }

        }

    public void rightTwister() throws InterruptedException
    {
        if (robot.opMode.gamepad2.right_bumper && robot.depositStage == 2)
        {
          robot.getTwister().setPosition(robot.getTwister().getPosition() + .1);
          Thread.sleep(120);

        }
    }

    public void leftTwister() throws InterruptedException{
        if(robot.opMode.gamepad2.left_bumper && robot.depositStage == 2)
        {
            robot.getTwister().setPosition(robot.getTwister().getPosition() - .1);
            Thread.sleep(120);

        }
    }

    public void intakeControl() throws InterruptedException {
        if (robot.opMode.gamepad1.x)
        {
            if (robot.intakeHeight == 1)
            {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
                Thread.sleep(200);
                robot.intakeHeight = 0;
            }
            else if ( robot.intakeHeight == 0)
            {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
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

    public void automaticPixel()
    {
        if (robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 2
                && robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 2
                && robot.depositStage == 0)
        {
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
            robot.getIntakeRoller().setPosition(robot.frontRollerStack);
            robot.depositStage = -1;
        }
    }

    public void flipIntake()
    {
        if (robot.opMode.gamepad2.a) {
            robot.getGrabber().setPosition(robot.grabberOpen);
            robot.getTwister().setPosition(robot.twisterPos4);
            robot.getDepositWrist().setPosition(robot.depositWristStore);
            robot.getArmLeft().setPosition(robot.armLeftStore);
            robot.getArmRight().setPosition(robot.armRightStore);

            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmRightTransfer);
            robot.getIntakeRoller().setPosition(robot.frontRollerReset);
        }
    }

    public void drop () throws Exception
    {
        if ((robot.opMode.gamepad2.right_trigger > .25 || robot.opMode.gamepad2.left_trigger > .25) && robot.depositStage == 2)
        {
            robot.getGrabber().setPosition(robot.grabberOpen);
            Thread.sleep(400);
            robot.getArmRight().setPosition(robot.armRightStore);
            robot.getArmLeft().setPosition(robot.armLeftStore);
            robot.getDepositWrist().setPosition(robot.depositWristStore);
            robot.getTwister().setPosition(robot.twisterPos4);
            Thread.sleep(1000);
            robot.getIntakeRoller().setPosition(robot.frontRollerGround);
            robot.depositStage = 0;
        }
    }
}