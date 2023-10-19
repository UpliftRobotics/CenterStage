package org.firstinspires.ftc.teamcode.Core.main;

import static org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath.atan2UL;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;

public class UpliftAutoImpl extends UpliftAuto {

    public UpliftRobot robot;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() {

    }


    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void goToPos(double finalX, double finalY, double finalAngle, double vel, double tol)
    {
        double deltaX = finalX - robot.worldX;
        double deltaY = finalY - robot.worldY;

        double dist = hypot(deltaX, deltaY);

        double relAngle = toDegrees(atan2UL(deltaY, deltaX));

        double deltaAngle = finalAngle - robot.worldAngle;
        double turnVal = deltaAngle / finalAngle;

        while(dist > tol)
        {
            double lfPow = sin(toRadians(relAngle) + (0.25 * PI)) * vel + turnVal;
            double rfPow = sin(toRadians(relAngle) - (0.25 * PI)) * vel - turnVal;
            double lbPow = sin(toRadians(relAngle) - (0.25 * PI)) * vel + turnVal;
            double rbPow = sin(toRadians(relAngle) + (0.25 * PI)) * vel - turnVal;

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
            robot.getLeftFront().setPower(lfPow / maxVal);
            robot.getLeftBack().setPower(lbPow / maxVal);
            robot.getRightBack().setPower(rbPow / maxVal);
            robot.getRightFront().setPower(rfPow / maxVal);

            deltaX = finalX - robot.worldX;
            deltaY = finalY - robot.worldY;

            dist = hypot(deltaX, deltaY);

            relAngle = toDegrees(atan2UL(deltaY, deltaX));

            deltaAngle = finalAngle - robot.worldAngle;
            turnVal = deltaAngle / finalAngle;

        }




    }


}
