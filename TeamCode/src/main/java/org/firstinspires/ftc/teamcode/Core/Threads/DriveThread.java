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

                extension();
                plane();
                intake(robot.intakePower);
                slides();


                // todo: validate user responsiveness and set sleep
                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();

            }
        }
    }
    public void intake(double input)
    {
        double power = input + 2 * -robot.opMode.gamepad2.left_stick_y;
        power = Range.clip(power , -1 , 1);
        robot.getIntake().setPower(power);
    }
    public void extension()
    {
        double power = -(robot.opMode.gamepad1.right_trigger - robot.opMode.gamepad1.left_trigger);


        if (power < -.1)
        {
            robot.extensionPower = 0;
            if (robot.getExtension().getCurrentPosition() < -850)
            {
                robot.getExtension().setPower(0);
            }
            else
            {
                robot.getExtension().setPower(power);
            }

        } else if (power > .1)
        {
            robot.extensionPower = 0;
            if (robot.getExtension().getCurrentPosition() > -50)
            {
                robot.getExtension().setPower(power * .1);

            }
            else
            {
                robot.getExtension().setPower(power);
            }
        }
        else
        {
            robot.getExtension().setPower(-robot.extensionPower);
        }

    }



    public void plane () throws InterruptedException
    {
        if (robot.opMode.gamepad1.dpad_up)
        {
            robot.getPlane().setPower(1);
            Thread.sleep(1000);
            robot.getPlane().setPower(0);
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

        robot.getFrontRight().setPower(speedFactor * (rfPow / maxVal));
        robot.getFrontLeft().setPower(speedFactor * (lfPow / maxVal));
        robot.getBackRight().setPower(speedFactor * (rbPow / maxVal));
        robot.getBackLeft().setPower(speedFactor * (lbPow / maxVal));
    }

    public void slides()
    {
        double power = .8 * robot.opMode.gamepad2.right_stick_y;
        if (robot.opMode.gamepad2.right_bumper)
            power = power * .05;

        if (robot.oneDriver)
        {
            power = -robot.slidePower;
        }


        // if going up stop from overextending

            if (power < 0.0) {
                if (robot.getSlideRight().getCurrentPosition() < -2200 || robot.getSlideLeft().getCurrentPosition() < -2200)
                {
                    robot.getSlideLeft().setPower( power * .1);
                    robot.getSlideRight().setPower(power * .1);
                } else {
                    robot.getSlideLeft().setPower(power);
                    robot.getSlideRight().setPower(power);
                }
            }
            // stop from overretracting
            else {
                if (robot.getSlideRight().getCurrentPosition() > 0 || robot.getSlideLeft().getCurrentPosition() > 0) {
                    robot.getSlideLeft().setPower(0);
                    robot.getSlideRight().setPower(0);
                } else {
                    robot.getSlideLeft().setPower(power);
                    robot.getSlideRight().setPower(power);
                }
            }

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
}
