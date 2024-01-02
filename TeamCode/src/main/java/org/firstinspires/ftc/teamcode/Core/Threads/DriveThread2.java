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
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.TurnPID;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

public class DriveThread2 extends Thread {
    private UpliftRobot robot;
    private static final String DRIVER_NAME2 = "DriverThread2Name";

    private boolean shutDown = false;

    public DriveThread2(UpliftRobot robot) {
        this.robot = robot;
    }


    @Override
    public void run() {
        while (!shutDown) {
            try {

//                alignWithBackDrop();

                reset();

                plane();

                extension();

                // todo: validate user responsiveness and set sleep
                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();

            }
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



    public void reset()
    {
        if (robot.opMode.gamepad1.left_bumper)
        {
            //lift front roller up, then while retracting the slides towards the robot, rotate the intake to the store
            //while continuing to rotate the intake motor inwards.
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
                double speed = pid.update(getAbsoluteAngle());

                robot.getFrontRight().setPower(speed);
                robot.getFrontLeft().setPower(-speed);
                robot.getBackRight().setPower(speed);
                robot.getBackLeft().setPower(-speed);
            }
            stopMotors();
        }
    }

    public double getAbsoluteAngle()
    {
        return robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void stopMotors() {
        robot.getFrontRight().setPower(0);
        robot.getFrontLeft().setPower(0);
        robot.getBackRight().setPower(0);
        robot.getBackLeft().setPower(0);
    }

    public void end() {
        shutDown = true;

        robot.opMode.telemetry.addData("Driver Thread2 stopped ", shutDown);

        robot.opMode.telemetry.update();

    }

    @Override
    public String toString() {
        return "DriveThread2{" +
                "name=" + DRIVER_NAME2 +
                '}';
    }


}
