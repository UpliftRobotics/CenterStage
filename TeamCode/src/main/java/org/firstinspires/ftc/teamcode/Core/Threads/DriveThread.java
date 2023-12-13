package org.firstinspires.ftc.teamcode.Core.Threads;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.TurnPID;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

public class DriveThread extends Thread {
    private UpliftRobot robot;
    private static final String DRIVER_NAME = "DriverThreadName";

    private boolean shutDown = false;

    public DriveThread(UpliftRobot robot) {
        this.robot = robot;
    }


    @Override
    public void run() {
        while (!shutDown) {
            try {
                double leftY = (Range.clip(-robot.opMode.gamepad1.left_stick_y, -1, 1));
                double rightX = (Range.clip(robot.opMode.gamepad1.right_stick_x, -1, 1));
                double leftX = (Range.clip(robot.opMode.gamepad1.left_stick_x, -1, 1));

                double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
                double magnitude = 0.8 * Range.clip(sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

                teleDrive(angle, magnitude, rightX, robot.opMode.gamepad1.right_bumper, robot);

//                robot.opMode.telemetry.addData("X", robot.worldX);
//                robot.opMode.telemetry.addData("Y", robot.worldY);
//                robot.opMode.telemetry.addData("X", robot.worldAngle);
//                robot.opMode.telemetry.update();

//                if(robot.opMode.gamepad1.dpad_right)
//                {
//                    robot.getLeftFront().setPower(0.5);
//                    robot.getRightFront().setPower(-0.5);
//                    robot.getLeftBack().setPower(-0.5);
//                    robot.getRightBack().setPower(0.5);
//                    Thread.sleep(3000);
//                }
//
//                if(robot.opMode.gamepad1.dpad_left)
//                {
//                    robot.getLeftFront().setPower(-0.5);
//                    robot.getRightFront().setPower(0.5);
//                    robot.getLeftBack().setPower(0.5);
//                    robot.getRightBack().setPower(-0.5);
//                    Thread.sleep(3000);
//                }

//                if(robot.opMode.gamepad1.dpad_up)
//                {
//                    robot.getLeftFront().setPower(0.5);
//                    robot.getRightFront().setPower(0.5);
//                    robot.getLeftBack().setPower(0.5);
//                    robot.getRightBack().setPower(0.5);
//                    Thread.sleep(3000);
//                }

//                if(robot.opMode.gamepad1.dpad_down)
//                {
//                    robot.getLeftFront().setPower(-0.5);
//                    robot.getRightFront().setPower(-0.5);
//                    robot.getLeftBack().setPower(-0.5);
//                    robot.getRightBack().setPower(-0.5);
//                    Thread.sleep(3000);
//                }

                alignWithBackDrop();


                plane();
                extension();

                // todo: validate user responsiveness and set sleep
                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();

            }
        }
    }

    public static void teleDrive(double joystickAngle, double speedVal,
                                 double turnVal, boolean slowModeInput, UpliftRobot robot) {
        double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
        double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
        double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
        double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;
//
        // find max total input out of the 4 motors
        double maxVal = abs(lfPow);
        if (abs(rfPow) > maxVal) {
            maxVal = abs(rfPow);
        }
        if (abs(lbPow) > maxVal) {
            maxVal = abs(lbPow);
        }
        if (abs(rbPow) > maxVal) {
            maxVal = abs(rbPow);
        }

        if (maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        float speedFactor = 1.0f;
        if (slowModeInput)
            speedFactor = 0.5f;

        robot.getLeftFront().setPower(speedFactor * (lfPow / maxVal));
        robot.getLeftBack().setPower(speedFactor * (lbPow / maxVal));
        robot.getRightBack().setPower(speedFactor * (rbPow / maxVal));
        robot.getRightFront().setPower(speedFactor * (rfPow / maxVal));
    }

    public void end() {
        shutDown = true;

        robot.opMode.telemetry.addData("Driver Thread stopped ", shutDown);

        robot.opMode.telemetry.update();

    }

    @Override
    public String toString() {
        return "DriveThread{" +
                "name=" + DRIVER_NAME +
                '}';
    }

    public void plane () throws InterruptedException
    {
        if (robot.opMode.gamepad1.dpad_up)
        {
            robot.getPlane().setPosition(.6);
        }
    }



    public void extension()
    {
        double power = .5 * (robot.opMode.gamepad1.right_trigger - robot.opMode.gamepad1.left_trigger);

        if (power > 0.0) {
            if (robot.getExtension().getCurrentPosition() > 850) {
                robot.getExtension().setPower(0);
            } else {
                robot.getExtension().setPower(power);
            }

        } else {
            if (robot.getExtension().getCurrentPosition() < 50) {
                robot.getExtension().setPower(0);

            } else {
                robot.getExtension().setPower(power);
            }
        }
    }



    public void alignWithBackDrop()
    {
        if(robot.opMode.gamepad1.a)
        {
            TurnPID pid = new TurnPID(90, 0.013, 0, 0.003);
            while(robot.opMode.opModeIsActive() && Math.abs(90 - getAbsoluteAngle()) > 1)
            {
                double motorPower = pid.update(getAbsoluteAngle());
                robot.getLeftFront().setPower(-motorPower);
                robot.getRightFront().setPower(motorPower);
                robot.getLeftBack().setPower(-motorPower);
                robot.getRightBack().setPower(motorPower);
            }
            stopMotors();
        }
    }

    public double getAbsoluteAngle()
    {
        return robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void stopMotors() {
        robot.getLeftFront().setPower(0);
        robot.getRightFront().setPower(0);
        robot.getLeftBack().setPower(0);
        robot.getRightBack().setPower(0);
    }
}
