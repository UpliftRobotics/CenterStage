package org.firstinspires.ftc.teamcode.Core.main;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.toolkit.Vision.CenterStageBlueClose;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.teamcode.Core.toolkit.Point;
import org.firstinspires.ftc.teamcode.Core.toolkit.Vision.CenterStageBlueFar;
import org.firstinspires.ftc.teamcode.Core.toolkit.Vision.CenterStageRedClose;
import org.firstinspires.ftc.teamcode.Core.toolkit.Vision.CenterStageRedFar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class UpliftRobot
{
    public Odometry odometry;
    DcMotor leftFront, rightFront, leftBack, rightBack, slideLeft, slideRight, extension, intake;
    Servo armLeft, armRight, grabberLeft, grabberRight, depositWrist, plane, intakeLinkLeft, IntakeLinkRight;

    ColorRangeSensor leftPixelDetector, rightPixelDetector;

    DistanceSensor leftAlign, rightAlign;

    public IMU imu;

    public CenterStageBlueClose pipelineBlueDepositSide;

    public CenterStageBlueFar pipelineBlueAudienceSide;
    public CenterStageRedClose pipelineRedDepositSide;
    public CenterStageRedFar pipelineRedAudienceSide;
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
    public static double robotEncoderWheelDistance = 8.7007;
    //12.73
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




    public int depositStage = 0;

    // v2
    public double armLeftPast = 1;
    public double armLeftGrab = 1;
    public double armLeftHold = .8;
    public double armLeftTransfer = .1;
    public double armLeftDrop = 0;

    public double armRightPast = 0;
    public double armRightGrab = .01;
    public double armRightHold = .2;
    public double armRightTransfer = .9;
    public double armRightDrop = 1;

    public double depositWristGrab = .21;
    public double depositWristHold = .45;
    public double depositWristTransfer = 0;
    public double depositWristDrop = .05;

    public double grabberLeftOpen = .15;
    public double grabberLeftClose = .48;
    public double grabberRightOpen = .35;
    public double grabberRightClose = 0;














    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
        odometry = new Odometry(this);
    }

    public void getHardware() {

        hardwareMap = opMode.hardwareMap;


        //wheels
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        //motors above drivetrain
        slideLeft = hardwareMap.get(DcMotor.class, "slide_left");
        slideRight = hardwareMap.get(DcMotor.class, "slide_right");
        extension = hardwareMap.get(DcMotor.class, "extension");
        intake = hardwareMap.get(DcMotor.class, "intake");


        //Servos
        depositWrist = hardwareMap.get(Servo.class, "deposit_wrist");

        plane = hardwareMap.get(Servo.class, "plane");

        grabberLeft = hardwareMap.get(Servo.class, "grabber_left");
        grabberRight = hardwareMap.get(Servo.class, "grabber_right");

        armLeft = hardwareMap.get(Servo.class, "arm_left");
        armRight = hardwareMap.get(Servo.class, "arm_right");

        //intakeLinkLeft = hardwareMap.get(Servo.class, "intake_left");
        //IntakeLinkRight = hardwareMap.get(Servo.class, "intake_right");

        //sensors
//        leftPixelDetector = hardwareMap.get(ColorRangeSensor.class, "leftPixelDetector");
        rightPixelDetector = hardwareMap.get(ColorRangeSensor.class, "color");
//
//        leftAlign = hardwareMap.get(DistanceSensor.class, "leftAlign");
//        rightAlign = hardwareMap.get(DistanceSensor.class, "rightAlign");





        initializeCamera();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public DcMotor getIntake() {
        return intake;
    }
    public Servo getDepositWrist() {
        return depositWrist;
    }
    public Servo getPlane(){return plane;}
    public Servo getIntakeLinkLeft(){return intakeLinkLeft;}
    public Servo getGetIntakeLinkRight(){return IntakeLinkRight;}
    public Servo getGrabberLeft(){return grabberLeft;}
    public Servo getGrabberRight(){return grabberRight;}
    public Servo getArmLeft(){return armLeft;}
    public Servo getArmRight(){return armRight;}



    public ColorRangeSensor getLeftPixelDetector()
    {
        return leftPixelDetector;
    }

    public ColorRangeSensor getRightPixelDetector() {
        return rightPixelDetector;
    }

    public DistanceSensor getLeftAlign() {
        return leftAlign;
    }
    public DistanceSensor getRightAlign() {
        return rightAlign;
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

             pipelineBlueDepositSide = new CenterStageBlueClose(opMode.telemetry);
             pipelineRedDepositSide = new CenterStageRedClose(opMode.telemetry);
             pipelineBlueAudienceSide = new CenterStageBlueFar(opMode.telemetry);
             pipelineRedAudienceSide = new CenterStageRedFar(opMode.telemetry);

               //changes this before each match depending on color and side

//               webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

               webcam.setPipeline(pipelineBlueDepositSide);


               if(!pipelineBlueDepositSide.blueClose)
               {
                   webcam.setPipeline(pipelineBlueAudienceSide);

                   if (!pipelineBlueAudienceSide.blueFar)
                   {
                       webcam.setPipeline(pipelineRedDepositSide);

                       if(!pipelineRedDepositSide.redClose)
                       {
                           webcam.setPipeline(pipelineRedAudienceSide);
                           
                       }

                   }

               }

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
