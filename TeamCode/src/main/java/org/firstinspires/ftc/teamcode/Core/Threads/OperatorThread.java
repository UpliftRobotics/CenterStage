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
                d1reset();
                rightTwister();
                leftTwister();
                intakeHeight();
                dropIntake();




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
            if (robot.getSlideRight().getCurrentPosition() > -30 || robot.getSlideLeft().getCurrentPosition() > -30) {
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
                if (robot.depositStage == 0)// intake on the ground for pick up, move inside the robot
                {
                    robot.getArmRight().setPosition(robot.armRightStore);
                robot.getArmLeft().setPosition(robot.armLeftStore);
                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
                robot.getTwister().setPosition(robot.twisterPos4);
                robot.getGrabber().setPosition(robot.grabberOpen);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
                robot.getIntakeRoller().setPosition(robot.frontRollerStore);

                Thread.sleep(1000);
                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
                robot.getArmLeft().setPosition(robot.armLeftTransfer);
                robot.getArmRight().setPosition(robot.armRightTransfer);
                Thread.sleep(500);
                robot.getGrabber().setPosition(robot.grabberClose2);
                Thread.sleep(200);
                robot.depositStage++;
                }
                else if (robot.depositStage == 1) //intake is in the robot, transfer by grabbing the pixels and then sending the intake out
                {
                    robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                robot.getArmRight().setPosition(robot.armRightDrop);
                robot.getArmLeft().setPosition(robot.armLeftDrop);
                Thread.sleep(200);
                robot.getDepositWrist().setPosition(robot.depositWristDrop);
                Thread.sleep(200);
                robot.depositStage++;

                }
                else if (robot.depositStage == 2) //move the pixels to scoring posistion and retract the intake
                {
                   robot.getGrabber().setPosition(robot.grabberOpen);
                   Thread.sleep(400);
                   robot.getArmRight().setPosition(robot.armRightStore);
                   robot.getArmLeft().setPosition(robot.armLeftStore);
                   robot.getDepositWrist().setPosition(robot.depositWristStore);
                   robot.getTwister().setPosition(robot.twisterPos4);
                   robot.depositStage = 0;
                }
            }

        }

    public void rightTwister() throws InterruptedException {
        if (robot.opMode.gamepad2.right_bumper && robot.depositStage == 3) {
            if(robot.getTwister().getPosition() == robot.twisterPos1) {
                robot.getTwister().setPosition(robot.twisterPos2);
                Thread.sleep(100);
            }
            else if(robot.getTwister().getPosition() == robot.twisterPos2) {
                robot.getTwister().setPosition(robot.twisterPos3);
                Thread.sleep(100);
            }
            else if(robot.getTwister().getPosition() == robot.twisterPos3) {
                robot.getTwister().setPosition(robot.twisterPos4);
                Thread.sleep(100);
            }
            else if(robot.getTwister().getPosition() == robot.twisterPos4) {
                robot.getTwister().setPosition(robot.twisterPos5);
                Thread.sleep(100);
            }
            else if(robot.getTwister().getPosition() == robot.twisterPos5) {
                robot.getTwister().setPosition(robot.twisterPos6);
                Thread.sleep(100);
            }
            else if(robot.getTwister().getPosition() == robot.twisterPos6) {
                robot.getTwister().setPosition(robot.twisterPos7);
                Thread.sleep(100);
            }
            else if(robot.getTwister().getPosition() == robot.twisterPos7) {
                robot.getTwister().setPosition(robot.twisterPos1);
                Thread.sleep(500);
            }

        }
    }

    public void leftTwister() throws InterruptedException{
        if(robot.opMode.gamepad2.left_bumper && robot.depositStage == 3){
            if (robot.getTwister().getPosition() == robot.twisterPos7) {
               robot.getTwister().setPosition(robot.twisterPos6);
               Thread.sleep(100);
            }
            if (robot.getTwister().getPosition() == robot.twisterPos6) {
                robot.getTwister().setPosition(robot.twisterPos5);
                Thread.sleep(100);
            }
            if (robot.getTwister().getPosition() == robot.twisterPos5) {
                robot.getTwister().setPosition(robot.twisterPos4);
                Thread.sleep(100);
            }
            if (robot.getTwister().getPosition() == robot.twisterPos4) {
                robot.getTwister().setPosition(robot.twisterPos3);
                Thread.sleep(100);
            }
            if (robot.getTwister().getPosition() == robot.twisterPos3) {
                robot.getTwister().setPosition(robot.twisterPos2);
                Thread.sleep(100);
            }
            if (robot.getTwister().getPosition() == robot.twisterPos2) {
                robot.getTwister().setPosition(robot.twisterPos1);
                Thread.sleep(100);
            }
            if (robot.getTwister().getPosition() == robot.twisterPos1) {
                robot.getTwister().setPosition(robot.twisterPos7);
                Thread.sleep(500);
            }

        }
    }

    public void intakeHeight() throws InterruptedException{
        if (robot.opMode.gamepad2.dpad_up && robot.depositStage == 0) {
            if (robot.intakeHeight == 0) {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
                Thread.sleep(150);
                robot.intakeHeight++;
            }
            if (robot.intakeHeight == 1) {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                Thread.sleep(150);
                robot.intakeHeight++;
            }
            if (robot.intakeHeight == 2) {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);
                Thread.sleep(150);
                robot.intakeHeight++;
            }
            if (robot.intakeHeight == 3) {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);
                Thread.sleep(150);
                robot.intakeHeight++;
            }
            if (robot.intakeHeight == 4) {
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
                Thread.sleep(150);
                robot.intakeHeight++;
            }
        }
        if(robot.opMode.gamepad2.dpad_down && robot.depositStage == 0){
            if(robot.intakeHeight == 5){
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);
                Thread.sleep(150);
                robot.intakeHeight--;
            }
            if(robot.intakeHeight == 4){
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack4);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack4);
                Thread.sleep(150);
                robot.intakeHeight--;
            }
            if(robot.intakeHeight == 3){
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                Thread.sleep(150);
                robot.intakeHeight--;
            }
            if(robot.intakeHeight == 2){
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack2);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack2);
                Thread.sleep(150);
                robot.intakeHeight--;
            }
            if(robot.intakeHeight == 1){
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
                robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
                Thread.sleep(150);
                robot.intakeHeight--;
            }
        }
    }

    public void dropIntake() throws InterruptedException{
        if(robot.opMode.gamepad2.dpad_left && robot.depositStage == 0)
        {
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftGround);
            robot.getIntakeArmRight().setPosition(robot.intakeArmRightGround);
            Thread.sleep(150);
            robot.intakeHeight = 0;
        }
    }






    public void reset() throws InterruptedException
    {

    }
}



