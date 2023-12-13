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

import java.util.concurrent.TimeUnit;

public class UpliftAutoImpl extends UpliftAuto {

    public UpliftRobot robot;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() throws InterruptedException
    {

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
    public void tranfer()
    {

        robot.getDepositWrist().setPosition(robot.depositWristTransfer);
        robot.getArmLeft().setPosition(robot.armLeftTransfer);
        robot.getArmRight().setPosition(robot.armRightTransfer);

    }

    public void deposit(int slidesDist, double slidesPower, boolean moveArm) throws InterruptedException
    {
        if (moveArm)
        {
            robot.getDepositWrist().setPosition(robot.depositWristDrop);
            robot.getArmLeft().setPosition(robot.armLeftDrop);
            robot.getArmRight().setPosition(robot.armRightDrop);
        }
        if (robot.getSlideRight().getCurrentPosition() < slidesDist)
        {
            while(robot.getSlideRight().getCurrentPosition() < slidesDist)
            {

                //negative power moves slides up
                robot.getSlideRight().setPower(-slidesPower);
                robot.getSlideLeft().setPower(-slidesPower);

            }
        }

        else
        {
            while(robot.getSlideRight().getCurrentPosition() > slidesDist)
            {

                //positive power moves slides down
                robot.getSlideRight().setPower(slidesPower);
                robot.getSlideLeft().setPower(slidesPower);

            }
        }





        robot.getSlideLeft().setPower(0);
        robot.getSlideRight().setPower(0);




    }
    public void drop() throws InterruptedException
    {
        robot.getGrabberLeft().setPosition(robot.grabberLeftOpen);
        robot.getGrabberRight().setPosition(robot.grabberRightOpen);

    }

    public void intake(double power) throws InterruptedException {
        double velocity = power;

        if(velocity > 0)
        {
            //intake
            int pixelsStored = 0;

            while(pixelsStored < 2)
            {
                robot.getIntake().setPower(velocity);
                if(robot.getPixelDetector().getDistance(DistanceUnit.CM) < 2)
                {
                    Thread.sleep(100);
                    pixelsStored++;
                }

            }
            robot.getIntake().setPower(0);
        }
        else
        {
            //outtake
            timer.startTime();
            double startTime = timer.seconds();

            while ((robot.getPixelDetector().alpha() == robot.getPixelDetector().red()) && (timer.seconds() - startTime < 3))
            {
                robot.getIntake().setPower(velocity);
            }
            while(robot.getPixelDetector().alpha() == robot.getPixelDetector().red())
            {
                velocity *= 1.1;
                robot.getIntake().setPower(velocity);
            }

            robot.getIntake().setPower(0);
        }

    }

    public void extensionPID(int extensionDist, int slowDownDistFromTargetPos, double extensionPower)
    {

        double minPower = 0.1;

        while(robot.getExtension().getCurrentPosition() < extensionDist)
        {
            double remainingDistance = extensionDist - robot.getExtension().getCurrentPosition();

            if(remainingDistance > slowDownDistFromTargetPos)
            {
                robot.getExtension().setPower(extensionPower);
            }
            else
            {
                // Adjust power based on remaining distance
                double slowdownFactor = remainingDistance / slowDownDistFromTargetPos;
                double slowedPower = minPower + (extensionPower - minPower) * slowdownFactor;

                // Set the slowed power to extension
                robot.getExtension().setPower(slowedPower);
            }

        }
    }

    public void reset() throws InterruptedException
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

        Thread.sleep(2000);

        while(robot.getSlideRight().getCurrentPosition() > 0)
        {

            //negative power moves slides up
            robot.getSlideRight().setPower(0.001);
            robot.getSlideLeft().setPower(0.001);

        }

        robot.getSlideLeft().setPower(0);
        robot.getSlideRight().setPower(0);
    }







}
