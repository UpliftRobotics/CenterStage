package org.firstinspires.ftc.teamcode.Core.main;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Core.toolkit.Vision.CenterStageBlue;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;
import org.firstinspires.ftc.teamcode.Core.toolkit.Vision.CenterStageRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class UpliftRobot
{
    public Odometry odometry;
    DcMotor leftFront, rightFront, leftBack, rightBack, slideLeft,slideRight, extension;
    Servo intakeAngleRight, intakeAngleLeft, depositArm, depositWrist, depositTwist, grabber, plane;
    CRServo intakeRoller;
    TouchSensor extensionTouch;
    public CenterStageBlue pipelineBlue;
    public CenterStageRed pipelineRed;
    public OpenCvCamera webcam;

    public double worldX;
    public double worldY;
    public double rawAngle;
    public double worldAngle;

    public static double COUNTS_PER_REV = 8192;
    //2048
    public static double wheelRadius = 19/25.4; // in inches
    public static double wheelCircumference = wheelRadius * (2 * Math.PI); // in inches
    public static double COUNTS_PER_INCH = COUNTS_PER_REV / wheelCircumference;
    //(720 * 4) / wheelCircumference;
    public static double robotEncoderWheelDistance = 12.73;
    public static double horizontalEncoderInchesPerDegreeOffset = 0;
    //0.0275
    //1.37480

    public static final Point blueLeftStack = new Point(67.5, 70);
    public static final Point blueMiddleStack = new Point(67.5, 70);
    public static final Point blueRightStack = new Point(67.5, 70);

    public static final Point redLeftStack = new Point(67.5, 70);
    public static final Point redMiddleStack = new Point(67.5, 70);
    public static final Point redRightStack = new Point(67.5, 70);

    // direction constants
    public static final int CLOCKWISE = 1;
    public static final int COUNTER_CLOCKWISE = 2;
    public static final int QUICKEST_DIRECTION = 0;



    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
//    OpenCvCamera webcam;

    // robot constants
    //inake pos
    public double intakeStorePos = .52;
    public double intakeGroundPos = .38 ;
    public double intake2Pixel = .428;
    public double intake3Pixel = .453;
    public double intake4Pixel = .478;
    public double intake5Pixel = .497;
    //deposit arm pos
    public double depositPick = .175;
    public double depositPick2 = 0.11;
    public double depositHold = .39;
    public double depositBack = .82;


    //deposit wrist pos
    public double wristPick = .475;
    public double wristPick2 = .32;
    public double wristHold = .5;
    public double wristBack = .15;

    //twister pos
    public double twistReset = .965;
    public double twistPosIncremnt = .25;

    //grabber pos
    public double grabberOpen = .55;
    public double grabberClose = 1;

    public int depositStage = 0;









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
        plane = hardwareMap.get(Servo.class, "plane");

        extensionTouch = hardwareMap.get(TouchSensor.class, "extension_touch");



        initializeCamera();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    public Servo getPlane(){return plane;}
    public TouchSensor getExtensionTouch()
    {
        return extensionTouch;
    }







    public void initializeCamera()
  {

       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


     webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
           public void onOpened()
           {
             webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);

               pipelineBlue = new CenterStageBlue(opMode.telemetry);
               pipelineRed = new CenterStageRed(opMode.telemetry);

               //changes this before each match depending on color
               webcam.setPipeline(pipelineRed);
               webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
           public void onError(int errorCode) {
            }
        });
    }

    public OpenCvCamera getWebcam() {
        return webcam;
    }
}
