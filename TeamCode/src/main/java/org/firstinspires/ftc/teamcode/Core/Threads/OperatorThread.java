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
                intake();
//                intakeAngle();
                slides();
                deposit();
                openGrabber();
//                rightTwister();
//                leftTwister();
                reset();
//                intakeDown();
//                intakeup();
//                closeGrabber();
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
        double power = .6 * robot.opMode.gamepad2.right_stick_y;
        if (robot.opMode.gamepad2.right_bumper)
            power  = power * .05;

        // if going up stop from overextending
        if (power < 0.0) {
            if (robot.getSlideRight().getCurrentPosition()  > 2800 || robot.getSlideLeft().getCurrentPosition() > 2800) {
                robot.getSlideLeft().setPower(0);
                robot.getSlideRight().setPower(0);
            } else {
                robot.getSlideLeft().setPower(power);
                robot.getSlideRight().setPower(power);
            }
        }
        // stop from overretracting
        else {
            if (robot.getSlideRight().getCurrentPosition() < 30 || robot.getSlideLeft().getCurrentPosition() < 30) {
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

    public void deposit() throws InterruptedException
    {
        if (robot.opMode.gamepad2.y)
            {
                if (robot.depositStage == 0)// intkae on the ground for pick up, move inside the robot
                {
                    robot.getIntakeRoller().setPosition(robot.frontRollerStore);
                    robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStore);
                    robot.getIntakeArmRight().setPosition(robot.intakeArmRightStore);
                    robot.getArmLeft().setPosition(robot.armLeftStore);
                    robot.getArmRight().setPosition(robot.armRightStore);
                    robot.depositStage++;
                    Thread.sleep(1000);
                }
                else if (robot.depositStage == 1) //intake is in the robot, transfer by grabbing the pixels and then sending the intake out
                {
                    //make sure that slides are in the robot
                    robot.getArmLeft().setPosition(robot.armLeftTransfer);
                    robot.getArmRight().setPosition(robot.armRightTransfer);
                    Thread.sleep(100);
                    robot.getIntake().setZeroPowerBehavior(FLOAT);
                    robot.getGrabber().setPosition(robot.grabberClose2); // has to be able to change if we have 1 or 2
                    Thread.sleep(150);
                    // send the slides out like 4 inches
                    robot.depositStage++;
                    Thread.sleep(1000);
                }
                else if (robot.depositStage == 2) //move the pixels to scoring posistion and retract the intake
                {
                    robot.getArmLeft().setPosition(robot.armLeftDrop);
                    robot.getArmRight().setPosition(robot.armRightDrop);
                    robot.getDepositWrist().setPosition(robot.depositWristDrop);
                    Thread.sleep(500);
                    // pull slides back in
                    robot.depositStage++;
                    Thread.sleep(1000);
                }
                else if (robot.depositStage == 3) //drop pixels, reset arm, reset intake , reset twister
                {
                   robot.getGrabber().setPosition(robot.grabberOpen);
                   Thread.sleep(500);
                   robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);
                   robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                   robot.getIntakeRoller().setPosition(robot.frontRollerGround);
                   robot.getArmLeft().setPosition(robot.armLeftStore);
                   robot.getArmRight().setPosition(robot.armRightStore);
                   robot.getDepositWrist().setPosition(robot.depositWristStore);
                   robot.depositStage = 0;
                   Thread.sleep(1000);
                }
            }

        }



    public void reset() throws InterruptedException
    {

    }
}



