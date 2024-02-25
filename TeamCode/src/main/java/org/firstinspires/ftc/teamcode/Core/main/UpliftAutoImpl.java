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

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.UpliftMath;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


import java.util.concurrent.TimeUnit;

public class UpliftAutoImpl extends UpliftAuto
{
//    AprilTagProcessor aprilProcessor = new AprilTagProcessor.Builder()
//
//            .setLensIntrinsics(822.317, 822.317, 319.495, 242.05)
//            .build();
//
//    VisionPortal portal = new VisionPortal.Builder()
//            .addProcessor(aprilProcessor)
//            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//            .setCameraResolution(new Size(640, 480))
//
//            .build();

    public UpliftRobot robot;
    public boolean goPark = false;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() throws InterruptedException {

    }


    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void stopMotors() {
        robot.getFrontRight().setPower(0);
        robot.getFrontLeft().setPower(0);
        robot.getBackRight().setPower(0);
        robot.getBackLeft().setPower(0);
    }


    public void driveToPosition(double xPosition, double yPosition, double movementSpeed, double tolerance, double targetAngle, int turnDirection) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();
        double startTime = timer.seconds();


        double xDistanceToPoint = xPosition - robot.worldX;
        double yDistanceToPoint = yPosition - robot.worldY;
        double distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);

        // use this to see how quickly to turn while driving to point
        double initialDistanceToPoint = distanceToPoint;
        double relativeAngle = toDegrees(UpliftMath.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;

        double approachZone = 20;

        while ((abs(distanceToPoint) > tolerance)) {
            if (timer.seconds() - startTime < 5) {
                driveTowards(UpliftMath.slowApproach(movementSpeed, distanceToPoint, approachZone, tolerance), relativeAngle, targetAngle, initialDistanceToPoint, turnDirection);

                xDistanceToPoint = xPosition - robot.worldX;
                yDistanceToPoint = yPosition - robot.worldY;
                distanceToPoint = hypot(xDistanceToPoint, yDistanceToPoint);
                relativeAngle = toDegrees(UpliftMath.atan2UL(yDistanceToPoint, xDistanceToPoint)) - robot.worldAngle;
                telemetry.addData("time", timer.seconds());
                telemetry.update();
            } else {
                goPark = true;
            }
        }

        // arrived at point, so stop
//        stopMotors();

        // correct angle to be preferred angle * DON'T NEED IF slowTurn() works correctly (within +-1 degree)
        turnTo(targetAngle, movementSpeed, QUICKEST_DIRECTION);

    }

    public void driveTowards(double speedVal, double relativeAngleToPoint, double targetAngle, double initialDistToPt, int turnDirection) {
        double turnVal = 0;
        double initialAngle = robot.worldAngle;
        double turnAngle = UpliftMath.angleRestrictions(targetAngle - initialAngle);

        if (turnAngle > 30) {
            if (turnDirection == CLOCKWISE) {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            } else if (turnDirection == COUNTER_CLOCKWISE) {
                turnVal = -Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            } else {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            }
        } else if (turnAngle > 5) {
            turnVal = 0.15;
        } else if (turnAngle < -30) {
            if (turnDirection == CLOCKWISE) {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            } else if (turnDirection == COUNTER_CLOCKWISE) {
                turnVal = -Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            } else {
                turnVal = Range.clip(60 / initialDistToPt * (180 / Math.abs(turnAngle)), -1, 1);
            }
        } else if (turnAngle < -5) {
            turnVal = -0.15;
        } else {
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
        if (abs(rf) > maxVal) {
            maxVal = abs(rf);
        }
        if (abs(lb) > maxVal) {
            maxVal = abs(lb);
        }
        if (abs(rb) > maxVal) {
            maxVal = abs(rb);
        }

        if (maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }


        robot.getFrontRight().setPower((rf / maxVal));
        robot.getFrontLeft().setPower((lf / maxVal));
        robot.getBackRight().setPower(rb / maxVal);
        robot.getBackLeft().setPower(lb / maxVal);
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

        robot.getFrontRight().setPower(-speed);
        robot.getFrontLeft().setPower(speed);
        robot.getBackRight().setPower(-speed);
        robot.getBackLeft().setPower(speed);
    }

    // method to turn a certain number of degrees ( [+] for clockwise and [-] for counter-clockwise )
    public void turn(double degrees, double speed) {
        double initialAngle = robot.rawAngle;
        // if turning counter-clockwise
        if (degrees < 0) {
            while (robot.rawAngle > (initialAngle + degrees)) {

                if (abs(degrees) < 10) {
                    spin(-0.3);
                } else if (abs(degrees) < 30) {
                    spin(-0.5);
                } else {
                    spin(-speed);
                }
            }
            // if turning clockwise
        } else if (degrees > 0) {
            while (robot.rawAngle < (initialAngle + degrees)) {

                if (abs(degrees) < 10) {
                    spin(0.3);
                } else if (abs(degrees) < 30) {
                    spin(0.5);
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
        if (quickestTurnAngle > 0) {
            if (directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle, speed);
            } else if (directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle - 360, speed);
            } else {
                turn(quickestTurnAngle, speed);
            }
        } else if (quickestTurnAngle < 0) {
            if (directionIndex == CLOCKWISE) {
                turn(quickestTurnAngle + 360, speed);
            } else if (directionIndex == COUNTER_CLOCKWISE) {
                turn(quickestTurnAngle, speed);
            } else {
                turn(quickestTurnAngle, speed);
            }
        } else {
            // NOTHING...
        }


    }

    public void transfer() {

        robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
        robot.getArmLeft().setPosition(robot.armLeftTransfer);
        robot.getArmRight().setPosition(robot.armRightTransfer);

    }

    public void deposit(int slidesDist, double slidesPower) throws InterruptedException {
        robot.getDepositWrist().setPosition(robot.depositWristDrop);
        robot.getArmLeft().setPosition(robot.armLeftDrop);
        robot.getArmRight().setPosition(robot.armRightDrop);
//        robot.getTwister().setPosition(twisterPos);

        while (abs(robot.getSlideRight().getCurrentPosition()) < slidesDist) {
            //negative power moves slides up
            robot.getSlideRight().setPower(-slidesPower);
            robot.getSlideLeft().setPower(-slidesPower);

            robot.opMode.telemetry.addData("right slide ticks", robot.getSlideRight().getCurrentPosition());
            robot.opMode.telemetry.addData("left slide ticks", robot.getSlideLeft().getCurrentPosition());
            robot.opMode.telemetry.update();


        }

        robot.getSlideLeft().setPower(0);
        robot.getSlideRight().setPower(0);

    }

    public void claw(String instance) throws InterruptedException {
        if (instance.equals("open")) {
            robot.getGrabber().setPosition(robot.grabberOpen);
        } else if (instance.equals("close1")) {
            robot.getGrabber().setPosition(robot.grabberClose1);
        } else if (instance.equals("close2")) {
            robot.getGrabber().setPosition(robot.grabberClose2);
        }

    }

    public void intake(double power) throws InterruptedException {
        double velocity = power;

        while (!(robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 1
                && robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 1)) {
            robot.getIntake().setPower(power);
        }
        robot.getIntake().setPower(0);
    }
//        if(velocity > 0)
//            //intake
//            if (robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 2
//                    && robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 2) {
//
//                robot.getIntake().setPower(velocity);
//
//                    telemetry.addData("left distance", robot.getPixelDetectorLeft().getDistance(DistanceUnit.MM));
//                    telemetry.addData("right distance", robot.getPixelDetectorRight().getDistance(DistanceUnit.MM));
//
//
//                    telemetry.update();
//                }
//
//            }
//            robot.getIntake().setPower(0);
//        }
//        else
//        {
//            //outtake
//            ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//            intakeTimer.startTime();
//            double startTime = intakeTimer.seconds();
//
//            while(((robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 2) || (robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 2)) && (intakeTimer.seconds() - startTime < 3))
//            {
//                robot.getIntake().setPower(velocity);
//            }
//            while((robot.getPixelDetectorRight().getDistance(DistanceUnit.CM) < 2) || (robot.getPixelDetectorLeft().getDistance(DistanceUnit.CM) < 2))
//            {
//                velocity *= 1.1;
//                robot.getIntake().setPower(velocity);
//            }
//        }
//
//        robot.getIntake().setPower(0);


    public void extensionPID(int extensionDist, int slowDownDist, double extensionPower) {
        robot.getExtension().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double minPower = 0.1;
        int realSlowDownDist = extensionDist - slowDownDist;

        while (robot.getExtension().getCurrentPosition() < extensionDist) {
            double remainingDistance = extensionDist - robot.getExtension().getCurrentPosition();

            if (remainingDistance > realSlowDownDist) {
                robot.getExtension().setPower(extensionPower);
            } else {
                // Adjust power based on remaining distance
                double slowdownFactor = remainingDistance / realSlowDownDist;
                double slowedPower = minPower + (extensionPower - minPower) * slowdownFactor;

//                double slowedPower = Math.max(minPower + (extensionPower - minPower) * slowdownFactor, extensionPower / 2);

                // Set the slowed power to extension
                robot.getExtension().setPower(slowedPower);
            }

        }
    }

    public void reset(boolean slidesDown, boolean extensionIn) throws InterruptedException {

        if (slidesDown) {

            while (robot.getSlideRight().getCurrentPosition() < 0) {

                //negative power moves slides up
                robot.getSlideRight().setPower(0.5);
                robot.getSlideLeft().setPower(0.5);

            }

            robot.getSlideLeft().setPower(0);
            robot.getSlideRight().setPower(0);

        }

        if (extensionIn) {
            while (robot.getExtension().getCurrentPosition() > 0) {

                robot.getSlideRight().setPower(-0.1);

            }

            robot.getExtension().setPower(0);
        }

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);

        Thread.sleep(200);

        robot.getIntakeRoller().setPosition(robot.frontRollerStore);

        Thread.sleep(200);

        robot.getGrabber().setPosition(robot.grabberOpen);

        Thread.sleep(200);

        robot.getDepositWrist().setPosition(robot.depositWristStore);

        Thread.sleep(200);

        robot.getArmLeft().setPosition(robot.armLeftStore);
        robot.getArmRight().setPosition(robot.armRightStore);

        Thread.sleep(200);

    }

//    public void cycles(String color, int numCycles) throws InterruptedException
//    {
//
//        double[] leftStackPos;
//        leftStackPos = new double[numCycles];
//
//        leftStackPos[0] = robot.intakeArmLeftStack5;
//        leftStackPos[1] = robot.intakeArmLeftStack4;
//        leftStackPos[2] = robot.intakeArmLeftStack3;
//        leftStackPos[3] = robot.intakeArmLeftStack2;
//        leftStackPos[4] = robot.intakeArmLeftGround;
//
//        double[] rightStackPos;
//        rightStackPos = new double[numCycles];
//
//        rightStackPos[0] = robot.intakeArmRightStack5;
//        rightStackPos[1] = robot.intakeArmRightStack4;
//        rightStackPos[2] = robot.intakeArmRightStack3;
//        rightStackPos[3] = robot.intakeArmRightStack2;
//        rightStackPos[4] = robot.intakeArmRightGround;
//
//        for(int i = 0; i < numCycles; i++)
//        {
//            //goes to intake position
//            if(color.equals("blue"))
//            {
//                driveToPosition(blueLeftStack.x, blueLeftStack.y, 0.5, blueLeftStack.angle);
//            }
//            else if (color.equals("red"))
//            {
//                driveToPosition(redRightStack.x, redRightStack.y, 0.5, redRightStack.angle);
//            }
//
//            //sets left and right intake arm positions
//            robot.getIntakeArmLeft().setPosition(leftStackPos[i]);
//            robot.getIntakeArmRight().setPosition(rightStackPos[i]);
//
//            Thread.sleep(200);
//
//            //extends to stack
//            extensionPID(600, 300, 0.5);
//
//            //sets roller position
//            robot.getIntakeRoller().setPosition(robot.frontRollerStack);
//
//            //intakes pixels
//            intake(0.2);
//
//            //pulls extension back in
//            reset(false, true);
//
//            //goes to deposit position
//            if(color.equals("blue"))
//            {
//                driveToPosition(3.5, 25, 0.7, 90, 2);
//            }
//            else if (color.equals("red"))
//            {
//                driveToPosition(4, 119, 0.7, 90);
//            }
//            Thread.sleep(1000);
//
//            //drops pixel
//            deposit(400, 0.1);
//            Thread.sleep(500);
//            claw("close2");
//
//            Thread.sleep(500);
//
//            //resets slides
//            reset(true, false);
//        }
//    }
//    public void cycles(String color, int numCycles) throws InterruptedException
//    {
//
//        double[] leftStackPos;
//        leftStackPos = new double[numCycles];
//
//        leftStackPos[0] = robot.intakeArmLeftStack5;
//        leftStackPos[1] = robot.intakeArmLeftStack4;
//        leftStackPos[2] = robot.intakeArmLeftStack3;
//        leftStackPos[3] = robot.intakeArmLeftStack2;
//        leftStackPos[4] = robot.intakeArmLeftReset;
//
//        double[] rightStackPos;
//        rightStackPos = new double[numCycles];
//
//        rightStackPos[0] = robot.intakeArmRightStack5;
//        rightStackPos[1] = robot.intakeArmRightStack4;
//        rightStackPos[2] = robot.intakeArmRightStack3;
//        rightStackPos[3] = robot.intakeArmRightStack2;
//        rightStackPos[4] = robot.intakeArmRightReset;
//
//        for(int i = 0; i < numCycles; i++)
//        {
//            //goes to intake position
//            if(color.equals("blue"))
//            {
//                driveToPosition(blueLeftStack.x, blueLeftStack.y, 0.5, blueLeftStack.angle);
//            }
//            else if (color.equals("red"))
//            {
//                driveToPosition(redRightStack.x, redRightStack.y, 0.5, redRightStack.angle);
//            }
//
//            //sets left and right intake arm positions
//            robot.getIntakeArmLeft().setPosition(leftStackPos[i]);
//            robot.getIntakeArmRight().setPosition(rightStackPos[i]);
//
//            Thread.sleep(200);
//
//            //extends to stack
//            extensionPID(600, 300, 0.5);
//
//            //sets roller position
//            robot.getIntakeRoller().setPosition(robot.frontRollerStack);
//
//            //intakes pixels
//            intake(0.2);
//
//            //pulls extension back in
//            reset(false, true);
//
//            //goes to deposit position
//            if(color.equals("blue"))
//            {
//                driveToPosition(3.5, 25, 0.7, 90, 2);
//            }
//            else if (color.equals("red"))
//            {
//                driveToPosition(4, 119, 0.7, 90);
//            }
//            Thread.sleep(1000);
//
//            //drops pixel
//            deposit(400, 0.1);
//            Thread.sleep(500);
//            claw("close2");
//
//            Thread.sleep(500);
//
//            //resets slides
//            reset(true, false);
//        }
//    }







    public void driveToAprilTag(double currentX, double currentY, double currentAngle) throws InterruptedException
    {

        double changeX = 0;
        double changeY = 0;
        double changeAngle = 0;

        AprilTagProcessor aprilProcessor = new AprilTagProcessor.Builder()

                .setLensIntrinsics(822.317, 822.317, 319.495, 242.05)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(aprilProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))

                .build();

        if(aprilProcessor.getDetections().size() > 0)
        {
            AprilTagDetection tag = aprilProcessor.getDetections().get(0);

            Thread.sleep(500);

            changeX = tag.ftcPose.y;
            changeY = tag.ftcPose.x;
            changeAngle = tag.ftcPose.yaw;

            telemetry.addData("Tag: ", tag.id);
            telemetry.addData("X: ", tag.ftcPose.x);
            telemetry.addData("Y: ", tag.ftcPose.y);
            telemetry.addData("Angle: ", tag.ftcPose.yaw);
            telemetry.update();
        }

        driveToPosition(currentX + changeX - 7, currentY +  changeY, 0.3, currentAngle + changeAngle);

    }
}





