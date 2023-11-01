package org.firstinspires.ftc.teamcode.Core.main;


import android.text.method.Touch;
import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class UpliftRobot
{
    public Odometry odometry;
    DcMotor leftFront, rightFront, leftBack, rightBack, slideLeft,slideRight, extension;
    Servo intakeAngleRight, intakeAngleLeft, depositArm, depositWrist, depositTwist, grabber;
    CRServo intakeRoller;
    TouchSensor extensionTouch;

    public double worldX;
    public double worldY;
    public double rawAngle;
    public double worldAngle;

    public static double wheelRadius = 19/25.4; // in inches
    public static double wheelCircumference = wheelRadius * (2 * Math.PI); // in inches
    public static double COUNTS_PER_INCH = (720 * 4) / wheelCircumference;
    public static double robotEncoderWheelDistance = 12.73;
    public static double horizontalEncoderInchesPerDegreeOffset = 0.0275;


    public static final Point blueLeftStack = new Point(67.5, 70);
    public static final Point blueMiddleStack = new Point(67.5, 70);
    public static final Point blueRightStack = new Point(67.5, 70);

    public static final Point redLeftStack = new Point(67.5, 70);
    public static final Point redMiddleStack = new Point(67.5, 70);
    public static final Point redRightStack = new Point(67.5, 70);



    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
//    OpenCvCamera webcam;




    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
        odometry = new Odometry(this);
    }

    public void getHardware() {

        hardwareMap = opMode.hardwareMap;

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        slideLeft = hardwareMap.get(DcMotor.class, "slide_left");
        slideRight = hardwareMap.get(DcMotor.class, "slide_right");
        extension = hardwareMap.get(DcMotor.class, "extension");

        intakeRoller = hardwareMap.get(CRServo.class, "intake_roller");
        intakeAngleRight = hardwareMap.get(Servo.class, "intake_angle_right");
        intakeAngleLeft = hardwareMap.get(Servo.class, "intake_angle_left");
        depositArm = hardwareMap.get(Servo.class, "deposit_arm");
        depositWrist = hardwareMap.get(Servo.class, "deposit_wrist");
        depositTwist = hardwareMap.get(Servo.class, "deposit_twist");
        grabber = hardwareMap.get(Servo.class, "grabber");

        extensionTouch = hardwareMap.get(TouchSensor.class, "extension_touch");










//        initializeCamera();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

//    public OpenCvCamera getWebcam()
//    {
//        return webcam;
//    }

    public DcMotor getLeftFront() {
        return leftFront;
    }
    public DcMotor getLeftBack() {
        return leftBack;
    }
    public DcMotor getRightBack() {
        return rightBack;
    }
    public DcMotor getRightFront() {
        return rightFront;
    }
    public DcMotor getSlideLeft() {
        return slideLeft;
    }
    public DcMotor getSlideRight() {
        return slideRight;
    }
    public DcMotor getExtension() {
        return extension;
    }
    public CRServo getIntakeRoller() {
        return intakeRoller;
    }
    public Servo getIntakeAngleRight() {
        return intakeAngleRight;
    }
    public Servo getIntakeAngleLeft() {
        return intakeAngleLeft;
    }
    public Servo getDepositArm() {
        return depositArm;
    }
    public Servo getDepositWrist() {
        return depositWrist;
    }
    public Servo getDepositTwist() {
        return depositTwist;
    }
    public Servo getGrabber() {
        return grabber;
    }
    public TouchSensor getExtensionTouch()
    {
        return extensionTouch;
    }







    //    public void initializeCamera()
//    {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened()
//            {
//
//                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
////
////                pipeline3 = new ConeAlignmentRed(opMode.telemetry);
////                webcam.setPipeline(pipeline3);
////                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
            }
//        });
//    }
//}
