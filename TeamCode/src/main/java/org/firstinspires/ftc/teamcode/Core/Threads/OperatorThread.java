package org.firstinspires.ftc.teamcode.Core.Threads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
                rightDrop();
                leftDrop();
//                rightTwister();
//                leftTwister();
                reset();
//                intakeDown();
//                intakeup();
//                closeGrabber();
                drop();


//                robot.opMode.telemetry.addData("magnet", robot.getMagnet().isPressed());
//                robot.opMode.telemetry.addData("odoRight" , robot.getOdoRight().getCurrentPosition());
//                robot.opMode.telemetry.update();


                // todo: validate user responsiveness and set sleep
//                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();

//                StringWriter sw = new StringWriter();
//                PrintWriter pw = new PrintWriter(sw);
//                e.printStackTrace(pw);
//
//                robot.opMode.telemetry.addData("Operator error ", e.getMessage());
//                robot.opMode.telemetry.addData("Operator error stack", sw.toString());
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

    public void deposit() throws InterruptedException {
        if (robot.opMode.gamepad2.y) {
            if (robot.depositStage == 0) {
                robot.getGrabberRight().setPosition(robot.grabberRightClose);
                robot.getGrabberLeft().setPosition(robot.grabberLeftClose);
                Thread.sleep(500);
//                robot.getDepositWrist().setPosition(robot.depositWristHold);
                robot.getArmLeft().setPosition(robot.armLeftHold);
                robot.getArmRight().setPosition(robot.armRightHold);
                robot.getDepositWrist().setPosition(robot.depositWristHold);
                Thread.sleep(200);
                robot.depositStage = 1;
            } else if (robot.depositStage == 1) {
                robot.getArmRight().setPosition(robot.armRightTransfer);
                robot.getArmLeft().setPosition(robot.armLeftTransfer);
                Thread.sleep(800);
                robot.getDepositWrist().setPosition(robot.depositWristTransfer);
                Thread.sleep(200);
                robot.depositStage = 2;
            } else if (robot.depositStage == 2) {
                robot.getArmRight().setPosition(robot.armRightDrop);
                robot.getArmLeft().setPosition(robot.armLeftDrop);
                robot.getDepositWrist().setPosition(robot.depositWristDrop);
                Thread.sleep(200);
                robot.depositStage = 3;
            }

        }

    }

    public void rightDrop() throws InterruptedException {
        if (robot.opMode.gamepad2.right_trigger > .5 && (robot.depositStage == 2 || robot.depositStage == 3)) {
            robot.getGrabberLeft().setPosition(robot.grabberLeftOpen);
        } else if (robot.opMode.gamepad2.right_trigger > .5 && robot.depositStage == 0 && robot.getGrabberLeft().getPosition() == robot.grabberLeftOpen) {
            robot.getGrabberLeft().setPosition(robot.grabberLeftClose);
            Thread.sleep(400);
        } else if (robot.opMode.gamepad2.right_trigger > .5 && robot.depositStage == 0 && robot.getGrabberLeft().getPosition() != robot.grabberLeftOpen) {
            robot.getGrabberLeft().setPosition(robot.grabberLeftOpen);
            Thread.sleep(400);
        }
    }

    public void leftDrop() throws InterruptedException {
        if (robot.opMode.gamepad2.left_trigger > .5 && (robot.depositStage == 2 || robot.depositStage == 3)) {
            robot.getGrabberRight().setPosition(robot.grabberRightOpen);
        } else if (robot.opMode.gamepad2.left_trigger > .5 && robot.depositStage == 0 && robot.getGrabberRight().getPosition() == robot.grabberRightOpen) {
            robot.getGrabberRight().setPosition(robot.grabberRightClose);
            Thread.sleep(400);
        } else if (robot.opMode.gamepad2.left_trigger > .5 && robot.depositStage == 0 && robot.getGrabberRight().getPosition() != robot.grabberRightOpen) {
            robot.getGrabberRight().setPosition(robot.grabberRightOpen);
            Thread.sleep(400);
        }
    }

    public void reset() throws InterruptedException
    {
        if (robot.opMode.gamepad2.dpad_down)
        {
            robot.getGrabberLeft().setPosition(robot.grabberLeftOpen);
            robot.getGrabberRight().setPosition(robot.grabberRightOpen);
            Thread.sleep(200);
            robot.getArmLeft().setPosition(robot.armLeftPast);
            robot.getArmRight().setPosition(robot.armRightPast);
            Thread.sleep(600);
            robot.getDepositWrist().setPosition(robot.depositWristGrab);
            Thread.sleep(500);
            robot.getArmLeft().setPosition(robot.armLeftGrab);
            robot.getArmRight().setPosition(robot.armRightGrab);
            robot.depositStage = 0;

        }
    }

    public void drop()
    {
        if (robot.opMode.gamepad2.a) {

            while (robot.getSlideRight().getCurrentPosition() < 600) {

                //negative power moves slides up
                robot.getSlideRight().setPower(-0.1);
                robot.getSlideLeft().setPower(-0.1);

            }

            robot.getSlideLeft().setPower(0);
            robot.getSlideRight().setPower(0);

            robot.getDepositWrist().setPosition(robot.depositWristDrop);
            robot.getArmLeft().setPosition(robot.armLeftDrop);
            robot.getArmRight().setPosition(robot.armRightDrop);
            robot.depositStage = 3;


        }
    }
}



