package org.firstinspires.ftc.teamcode.Core.main;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

    //Drive Motors
    DcMotor frontLeft, frontRight, backLeft, backRight;

    //Slide Motors
    DcMotor slideLeft, slideRight;

    //Other Motors
    DcMotor extension, intake;

    //Solo Servos
    Servo grabber, twister, depositWrist, intakeRoller;

    //Dual Servos
    Servo armLeft, armRight, intakeArmLeft, intakeArmRight;

    CRServo plane;

    ColorRangeSensor pixelDetectorLeft, pixelDetectorRight;

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

    public static final Point blueLeftStack = new Point(50, 50, 93);
    public static final Point blueMiddleStack = new Point(0, 0, 0);
    public static final Point blueRightStack = new Point(0, 0, 0);

    public static final Point redLeftStack = new Point(0, 0, 0);
    public static final Point redMiddleStack = new Point(0, 0, 0);
    public static final Point redRightStack = new Point(67.5, 70, 93);

    // direction constants
    public static final int CLOCKWISE = 1;
    public static final int COUNTER_CLOCKWISE = 2;
    public static final int QUICKEST_DIRECTION = 0;



    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
//    OpenCvCamera webcam;

    // robot constants
    //inake pos




    public int depositStage = 0;
    // 0 = intaking pixels
    // 1 = pixels will be transfered to the grabber
    // 2 = pixels will be prepped for dropping
    // 3 = pixels will drop and reset arm back inside and folds intake back out

    public int intakeHeight = 0;
    public double intakePower = 0;
    public double extensionPower = 0;
    public boolean oneDriver = false;
    public double slidePower = 0;




    public double depositWristStore = .05;
    public double depositWristTransfer1 = .13;
    public double depositWristTransfer2 = .24;
    public double depositWristDrop = .28;

    public double frontRollerStore = 0.82;
    public double frontRollerGround = .495;
    public double frontRollerReset = .5;
    public double frontRollerStack = .6;

    public double intakeArmLeftStore = 0.02;
    public double intakeArmLeftTransfer = .14;
    public double intakeArmLeftStack5 = .81;
    public double intakeArmLeftStack4 = .83;
    public double intakeArmLeftStack3 = .86;
    public double intakeArmLeftStack2 = .89;
    public double intakeArmLeftReset = .9;
    public double intakeArmLeftGround = .93;
    public double intakeArmLeftGround1 = .91;


    public double intakeArmRightStore = .98;
    public double intakeArmRightTransfer = .86;
    public double intakeArmRightStack5 = .19;
    public double intakeArmRightStack4 = .17;
    public double intakeArmRightStack3 = .14;
    public double intakeArmRightStack2 = .11;
    public double intakeArmRightGround = .07;

    public double intakeArmRightGround1 = .09;
    public double intakeArmRightReset = .1;

    public double armLeftStore = 0.79;
    public double armLeftTransfer = 0.685;
    public double armLeftDrop = .08;
    public double armLeftReset = .5;

    public double armRightStore = 0.21;
    public double armRightTransfer = 0.325;
    public double armRightDrop = .92;
    public double armRightReset = .5;


    public double grabberClose1 = .69;
    public double grabberClose2 = .655;

    public double grabberOpen = .49;

    public double twisterPos4 = .532;














    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
        odometry = new Odometry(this);
    }

    public void getHardware() {

        hardwareMap = opMode.hardwareMap;


        //wheels
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");

        //motors above drivetrain
        slideLeft = hardwareMap.get(DcMotor.class, "slide_left");
        slideRight = hardwareMap.get(DcMotor.class, "slide_right");

        extension = hardwareMap.get(DcMotor.class, "extension");
        intake = hardwareMap.get(DcMotor.class, "intake");


        //Servos
        depositWrist = hardwareMap.get(Servo.class, "deposit_wrist");
        plane = hardwareMap.get(CRServo.class, "plane");
        grabber = hardwareMap.get(Servo.class, "grabber");

        armLeft = hardwareMap.get(Servo.class, "arm_left");
        armRight = hardwareMap.get(Servo.class, "arm_right");

        intakeArmLeft = hardwareMap.get(Servo.class, "intake_arm_left");
        intakeArmRight = hardwareMap.get(Servo.class, "intake_arm_right");

        twister = hardwareMap.get(Servo.class, "twister");
        intakeRoller = hardwareMap.get(Servo.class, "intake_roller");

        //sensors
        pixelDetectorLeft = hardwareMap.get(ColorRangeSensor.class, "pixel_detector_left");
        pixelDetectorRight = hardwareMap.get(ColorRangeSensor.class, "pixel_detector_right");









        initializeCamera();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

//    public OpenCvCamera getWebcam()
//    {
//        return webcam;
//    }

    public DcMotor getFrontRight() {
        return frontRight;
    }
    public DcMotor getFrontLeft() {
        return frontLeft;
    }
    public DcMotor getBackRight() {
        return backRight;
    }

    public DcMotor getBackLeft(){
        return backLeft;
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
    public CRServo getPlane(){return plane;}

    public Servo getGrabber(){return grabber;}


    public Servo getArmLeft(){return armLeft;}
    public Servo getArmRight(){return armRight;}

    public Servo getTwister()
    {
        return twister;
    }
    public Servo getIntakeRoller()
    {
        return intakeRoller;
    }
    public Servo getIntakeArmLeft()
{
    return intakeArmLeft;
}
    public Servo getIntakeArmRight()
    {
        return intakeArmRight;
    }


    public ColorRangeSensor getPixelDetectorLeft()
    {
        return pixelDetectorLeft;
    }

    public ColorRangeSensor getPixelDetectorRight() {
        return pixelDetectorRight;
    }


    public void initializeCamera()
  {

       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


     webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
           public void onOpened()
           {
//             webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);

             pipelineBlueDepositSide = new CenterStageBlueClose(opMode.telemetry);
             pipelineRedDepositSide = new CenterStageRedClose(opMode.telemetry);
             pipelineBlueAudienceSide = new CenterStageBlueFar(opMode.telemetry);
             pipelineRedAudienceSide = new CenterStageRedFar(opMode.telemetry);

               //changes this before each match depending on color and side

               webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

               webcam.setPipeline(pipelineBlueAudienceSide);

//               if(!pipelineBlueDepositSide.blueClose)
//               {
//                   webcam.setPipeline(pipelineBlueAudienceSide);
//
//
//                   if (!pipelineBlueAudienceSide.blueFar)
//                   {
//                       webcam.setPipeline(pipelineRedDepositSide);
//
//                       if(!pipelineRedDepositSide.redClose)
//                       {
//                           webcam.setPipeline(pipelineRedAudienceSide);
//
//                       }
//
//                   }
//
//               }
//               opMode.telemetry.addData("blueClose:", pipelineBlueDepositSide.blueClose);
//               opMode.telemetry.addData("blueFar:", pipelineBlueAudienceSide.blueFar);
//               opMode.telemetry.addData("redClose:", pipelineRedDepositSide.redClose);
//               opMode.telemetry.addData("redFar:", pipelineRedAudienceSide.redFar);
//               opMode.telemetry.update();


           }

            @Override

           public void onError(int errorCode)
            {

            }
        });
    }

    public OpenCvCamera getWebcam() {
        return webcam;
    }
}
