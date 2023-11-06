package org.firstinspires.ftc.teamcode.Core.main;

import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.CLOCKWISE;
import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.COUNTER_CLOCKWISE;
import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.QUICKEST_DIRECTION;
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
import com.qualcomm.robotcore.util.Range;

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

    public void stopMotors() {
        robot.getLeftFront().setPower(0);
        robot.getRightFront().setPower(0);
        robot.getLeftBack().setPower(0);
        robot.getRightBack().setPower(0);
    }

//
//    public void goToPos(double finalX, double finalY, double finalAngle, double vel, double tol)
//    {
//        double deltaX = finalX - robot.worldX;
//        double deltaY = finalY - robot.worldY;
//
//        double dist = hypot(deltaX, deltaY);
//
//        double relAngle = toDegrees(atan2UL(deltaY, deltaX)) - robot.worldAngle;
//
//        double deltaAngle = finalAngle - robot.worldAngle;
////        double turnVal = deltaAngle / finalAngle;
//        double turnVal = 0;
//
//        while(abs(dist) > tol)
//        {
//            double lfPow = sin(toRadians(90 - relAngle) + (0.25 * PI)) * vel + turnVal;
//            double rfPow = sin(toRadians(90 - relAngle) - (0.25 * PI)) * vel - turnVal;
//            double lbPow = sin(toRadians(90 - relAngle) - (0.25 * PI)) * vel + turnVal;
//            double rbPow = sin(toRadians(90 - relAngle) + (0.25 * PI)) * vel - turnVal;
//
//            // find max total input out of the 4 motors
//            double maxVal = abs(lfPow);
//            if (abs(rfPow) > maxVal) {
//                maxVal = abs(rfPow);
//            }
//            if (abs(lbPow) > maxVal) {
//                maxVal = abs(lbPow);
//            }
//            if (abs(rbPow) > maxVal) {
//                maxVal = abs(rbPow);
//            }
//
//            if (maxVal < (1 / sqrt(2))) {
//                maxVal = 1 / sqrt(2);
//            }
//
//            // set the scaled powers
//            robot.getLeftFront().setPower(lfPow / maxVal);
//            robot.getLeftBack().setPower(lbPow / maxVal);
//            robot.getRightBack().setPower(rbPow / maxVal);
//            robot.getRightFront().setPower(rfPow / maxVal);
//
//            deltaX = finalX - robot.worldX;
//            deltaY = finalY - robot.worldY;
//
//            dist = hypot(deltaX, deltaY);
//
//            relAngle = toDegrees(atan2UL(deltaY, deltaX)) - robot.worldAngle;
//
//            deltaAngle = finalAngle - robot.worldAngle;
////            turnVal = deltaAngle / finalAngle;
//            turnVal = 0;
//        }

    public void driveToPosition(double xPosition, double yPosition, double movementSpeed, double tolerance, double targetAngle, int turnDirection) {
        double xDistanceToPoint = xPosition - robot.worldX;
        double yDistanceToPoint = yPosition - robot.worldY;
        double distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);

        // use this to see how quickly to turn while driving to point
        double initialDistanceToPoint = distanceToPoint;
        double relativeAngle = toDegrees(UpliftMath.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;

        double approachZone = 20;

        while (abs(distanceToPoint) > tolerance) {

            driveTowards(UpliftMath.slowApproach(movementSpeed, distanceToPoint, approachZone, tolerance), relativeAngle, targetAngle, initialDistanceToPoint, turnDirection);

            xDistanceToPoint = xPosition - robot.worldX;
            yDistanceToPoint = yPosition - robot.worldY;
            distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);
            relativeAngle = toDegrees(UpliftMath.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;
        }

        // arrived at point, so stop
        stopMotors();

        // correct angle to be preferred angle * DON'T NEED IF slowTurn() works correctly (within +-1 degree)
        turnTo(targetAngle, movementSpeed, QUICKEST_DIRECTION);

    }
        public void driveTowards(double speedVal, double relativeAngleToPoint, double targetAngle, double initialDistToPt, int turnDirection) {
            double turnVal = 0;
            double initialAngle = robot.worldAngle;
            double turnAngle = UpliftMath.angleRestrictions(targetAngle - initialAngle);

            if(turnAngle > 30)
            {
                if(turnDirection == CLOCKWISE)
                {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
                }
                else if(turnDirection == COUNTER_CLOCKWISE)
                {
                turnVal = -Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
                }
                else
                {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
                }
            }
            else if(turnAngle > 5)
            {
                turnVal = 0.15;
            }
            else if(turnAngle < -30)
            {
                if(turnDirection == CLOCKWISE)
                {
                    turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
                } else if(turnDirection == COUNTER_CLOCKWISE)
                {
                    turnVal = -Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
                } else {
                    turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
                }
            }
            else if(turnAngle < -5)
            {
                turnVal = -0.15;
            }
            else
            {
                turnVal = 0;
        }

//        if(turnAngle > 10) {
//            if (turnDirection == CLOCKWISE) {
//                turnVal = Range.clip(MathFunctions.slowTurnDriving(1, turnAngle, 30, 10), -1, 1);
//            } else if (turnDirection == COUNTER_CLOCKWISE) {
//                turnVal = -Range.clip(MathFunctions.slowTurnDriving(1, turnAngle - 360, 30, 10), -1, 1);
//            } else {
//                turnVal = Range.clip(MathFunctions.slowTurnDriving(1, turnAngle, 30, 10), -1, 1);
//            }
//        } else if(turnAngle > 5)
//        } else if(turnAngle < -10) {
//            if(turnDirection == CLOCKWISE) {
//                turnVal = Range.clip(MathFunctions.slowTurnDriving(1, turnAngle + 360, 30, 10), -1, 1);
//            } else if(turnDirection == COUNTER_CLOCKWISE) {
//                turnVal = -Range.clip(MathFunctions.slowTurnDriving(1, turnAngle, 30, 10), -1, 1);
//            } else {
//                turnVal = -Range.clip(MathFunctions.slowTurnDriving(1, turnAngle, 30, 10), -1, 1);
//            }
//        } else if(turnAngle < -5) {
//            // NOTHING...
//        }

        double lf = sin(toRadians(90 - relativeAngleToPoint) + (0.25 * PI)) * speedVal + turnVal;
        double rf = sin(toRadians(90 - relativeAngleToPoint) - (0.25 * PI)) * speedVal - turnVal;
        double lb = sin(toRadians(90 - relativeAngleToPoint) - (0.25 * PI)) * speedVal + turnVal;
        double rb = sin(toRadians(90 - relativeAngleToPoint) + (0.25 * PI)) * speedVal - turnVal;

        // find max total input out of the 4 motors
        double maxVal = abs(lf);
        if(abs(rf) > maxVal){
            maxVal = abs(rf);
        }
        if(abs(lb) > maxVal){
            maxVal = abs(lb);
        }
        if(abs(rb) > maxVal){
            maxVal = abs(rb);
        }

        if(maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }


        // set the scaled powers
        robot.getLeftFront().setPower(1 * (lf / maxVal));
        robot.getRightFront().setPower(1 * (rf / maxVal));
        robot.getLeftBack().setPower(lb / maxVal);
        robot.getRightBack().setPower(rb / maxVal);
    }

    public void driveToPosition(double xPosition, double yPosition, double movementSpeed, double targetAngle, int turnDirection) {
        driveToPosition(xPosition, yPosition, movementSpeed, 1, targetAngle, turnDirection);
    }

    public void driveToPosition(double xPosition, double yPosition, double movementSpeed, double targetAngle) {
        driveToPosition(xPosition, yPosition, movementSpeed, 1, targetAngle, QUICKEST_DIRECTION);
    }


    // method to constantly spin ( [+] for clockwise and [-] for counter-clockwise )
    public void spin(double speed) {
        speed = Range.clip(speed, -1, 1);

        robot.getLeftFront().setPower(speed);
        robot.getRightFront().setPower(-speed);
        robot.getLeftBack().setPower(speed);
        robot.getRightBack().setPower(-speed);
    }

    // method to turn a certain number of degrees ( [+] for clockwise and [-] for counter-clockwise )
    public void turn(double degrees, double speed) {
        double initialAngle = robot.rawAngle;
        // if turning counter-clockwise
        if(degrees < 0) {
            while(robot.rawAngle > (initialAngle + degrees)) {

                if(abs(degrees) < 10) {
                    spin(-0.2);
                } else if(abs(degrees) < 30) {
                    spin(-0.4);
                } else {
                    spin(-speed);
                }
            }
            // if turning clockwise
        } else if(degrees > 0) {
            while(robot.rawAngle < (initialAngle + degrees)) {

                if(abs(degrees) < 10) {
                    spin(0.2);
                } else if(abs(degrees) < 30) {
                    spin(0.4);
                } else {
                    spin(speed);
                }
            }
        } else {
            // either a value of 0 was passed into the method, or some null/NA value [do nothing]
        }

        stopMotors();

    }

    // method to turn TO a certain angle (within the angle restrictions), with either the shortest path (technique 0) or through a specified direction in the direction indicator (clockwise for 1, counter-clockwise for 2)
    public void turnTo(double targetAngle, double speed, int directionIndex) {
        double initialAngle = robot.worldAngle;
        double quickestTurnAngle = UpliftMath.angleRestrictions(targetAngle - initialAngle);
        if(quickestTurnAngle > 0) {
            if(directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle, speed);
            } else if(directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle - 360, speed);
            } else {
                turn(quickestTurnAngle, speed);
            }
        } else if(quickestTurnAngle < 0) {
            if(directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle + 360, speed);
            } else if(directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle, speed);
            } else {
                turn(quickestTurnAngle, speed);
            }
        } else {
            // NOTHING...
        }

    }





}
