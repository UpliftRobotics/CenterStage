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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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

public class UpliftAutoImpl extends UpliftAuto {

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel

    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 4.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.015;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.02;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 4;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


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

//        double pow = abs(extensionPower);

        double minPower = 0.1;
        int realSlowDownDist = extensionDist - slowDownDist;

        while (robot.getExtension().getCurrentPosition() < extensionDist) {
            telemetry.addData("extensionPos", robot.getExtension().getCurrentPosition());
            telemetry.update();

            double remainingDistance = extensionDist - robot.getExtension().getCurrentPosition();

            if (remainingDistance > realSlowDownDist) {
                robot.getExtension().setPower(extensionPower);
            } else {
                // Adjust power based on remaining distance
                double slowdownFactor = remainingDistance / realSlowDownDist;
                double slowedPower = minPower + (extensionPower - minPower) * slowdownFactor;

                // Set the slowed power to extension
                robot.getExtension().setPower(slowedPower);
            }

        }
    }

    public void extension(int dist, double pow) {
        int extensionDist = -dist;

        while (robot.getExtension().getCurrentPosition() > extensionDist) {
            robot.getExtension().setPower(-pow);

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


    public void driveToAprilTag(int goalTag, double goalDistance ) {
        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (opModeIsActive())
        {

            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == goalTag))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData(">", "Drive using joysticks to find valid target\n");
            }

//            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound)
            {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - goalDistance);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
                sleep(10);
                if (desiredTag.ftcPose.range < goalDistance + 5)
                {
                    stopMotors();
                    break;
                }

            }

        }


    }


        public void moveRobot ( double x, double y, double yaw){
            // Calculate wheel powers.
            double leftFrontPower = x - y - yaw;
            double rightFrontPower = x + y + yaw;
            double leftBackPower = x + y - yaw;
            double rightBackPower = x - y + yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.

            leftFrontDrive.setPower(rightBackPower);
            rightFrontDrive.setPower(leftBackPower);
            leftBackDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(leftFrontPower);
        }

        private void initAprilTag () {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
        private void setManualExposure ( int exposureMS, int gain){
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested()) {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }
}






