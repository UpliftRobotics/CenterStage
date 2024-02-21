package org.firstinspires.ftc.teamcode.Core.programs.Autos.NoCycles;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Blue Audience 2 + 1", group = "Opmodes")
public class BlueAudience2Plus1 extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware()
    {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException {

        claw("open");

        robot.getDepositWrist().setPosition(robot.depositWristStore);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);

        robot.getTwister().setPosition(robot.twisterPos4);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getArmLeft().setPosition(robot.armLeftStore);

        robot.getIntake().setPower(0.1);
        Thread.sleep(1000);

//        claw("close1");-

        robot.frontWebcam.setPipeline(robot.pipelineBlueAudienceSide);


    }

    @Override
    public void body() throws InterruptedException
    {
        int location = robot.pipelineBlueAudienceSide.location;
        robot.frontWebcam.closeCameraDevice();

        AprilTagProcessor aprilProcessor = new AprilTagProcessor.Builder()

                .setLensIntrinsics(822.317, 822.317, 319.495, 242.05)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(aprilProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))

                .build();


        odom.setOdometryPosition(100, 0, 0);
        sleep(5000);

//            robot.getExtension().setPower(-.1);
//            //left
//            if(location == 0 || location == -1 )
//            {
//
////                drop position
//                driveToPosition(100, 15, 0.7, 0);
//                driveToPosition(96, 31, 0.5, -45, 2);
//
//                driveToPosition(104, 16, 0.5, -45, 2);
//                driveToPosition(104, 50, 0.7, 0);
//                driveToPosition(105, 58, 0.7, 90);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);
//
//                Thread.sleep(1000);
//
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//
//                driveToPosition(112, 57.5, 0.5, 90);
//
//                Thread.sleep(2000);
//
//               // robot.getIntakeRoller().setPosition(robot.frontRollerGround);
//
//               // intake(.7);
//
//                driveToPosition(75, 60, 0.9, 90);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntake().setPower(.5);
//                Thread.sleep(1000);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
//                robot.getArmLeft().setPosition(robot.armLeftTransfer);
//                robot.getArmRight().setPosition(robot.armRightTransfer);
//                Thread.sleep(500);
//
//                claw("close2");
//
//                robot.getIntake().setPower(0);
//
//                driveToPosition(75, 60, 0.9, 90);
//
//                driveToPosition(30, 57, 0.9, 90);
//
//                robot.getIntake().setPower(-0.5);
//                Thread.sleep(200);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(15, 45, 0.9, 92);
//                driveToPosition(10, 40, 0.9, 92);
//
////                Thread.sleep(1000);
//                driveToPosition(6, 34, 0.9, 95);
//
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightReset);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftReset);
//
//                Thread.sleep(2000);
//
//                deposit(200, 0.5);
//                robot.getTwister().setPosition(0.9);
//
//                Thread.sleep(1000);
//
//                claw("open");
//                Thread.sleep(1000);
//
//                driveToPosition(10, 34, 0.8, 95);
//
//                robot.getTwister().setPosition(robot.twisterPos4);
//
//                reset(true, false);
//
//                Thread.sleep(1000);
//
//
//            }
//
//            //middle
//            if(location == 1 )
//            {
//
//
////                drop position
//                driveToPosition(101, 32, 0.8, 0);
//
//                driveToPosition(102, 20, 0.9, 0);
//                driveToPosition(114, 30, 0.9, 0);
//                driveToPosition(113, 50, 0.9, 0);
//                driveToPosition(113, 58, 0.9, 90);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);
//
//                Thread.sleep(1000);
//
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//
//                driveToPosition(117, 55, 0.5, 92);
//
//                Thread.sleep(2000);
//
//            //   robot.getIntakeRoller().setPosition(robot.frontRollerGround);
//
//              //  intake(.7);
//
//                driveToPosition(75, 62, 0.9, 90);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntake().setPower(.5);
//                Thread.sleep(1000);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
//                robot.getArmLeft().setPosition(robot.armLeftTransfer);
//                robot.getArmRight().setPosition(robot.armRightTransfer);
//                Thread.sleep(500);
//
//                claw("close2");
//
//                robot.getIntake().setPower(0);
//
//
//
//                driveToPosition(25, 60, 0.9, 90);
//
//                robot.getIntake().setPower(-0.5);
//                Thread.sleep(200);
//                robot.getIntake().setPower(0);
//
//                driveToPosition(15, 56, 0.8, 92);
//                driveToPosition(9, 50, 0.8, 92);
//
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightReset);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftReset);
//
//                Thread.sleep(2000);
//
//                deposit(200, 0.5);
//                robot.getTwister().setPosition(0.9);
//
//                Thread.sleep(1000);
//
//                claw("open");
//                Thread.sleep(1000);
//
//                driveToPosition(12, 45, 0.8, 90);
//                robot.getTwister().setPosition(robot.twisterPos4);
//
//                reset(true, false);
//
//                Thread.sleep(1000);
//
//            }
//
//            // right
//
//            if(location == 2 )
//            {
//                //drop position
//                driveToPosition(110, 28, 0.7, 0);
//
//                driveToPosition(109, 15, 0.7, 0);
//                driveToPosition(96, 15, 1, 0);
//                driveToPosition(100, 50, 0.85, 0);
//
//                driveToPosition(105, 57, 0.85, 88);
//
//                Thread.sleep(1000);
//
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack5);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack5);
//
//                Thread.sleep(1000);
//
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer1);
//
//                driveToPosition(111, 59, 0.5, 90);
//
//                Thread.sleep(2000);
//
////                robot.getIntakeRoller().setPosition(robot.frontRollerGround);
////
////                intake(.7);
//
//                driveToPosition(75, 58, 0.7, 85);
//
//                robot.getIntakeRoller().setPosition(robot.frontRollerStore);
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
//                robot.getIntake().setPower(.5);
//                Thread.sleep(1000);
//                robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
//                robot.getArmLeft().setPosition(robot.armLeftTransfer);
//                robot.getArmRight().setPosition(robot.armRightTransfer);
//                Thread.sleep(500);
//
//                claw("close1");
//
//                robot.getIntake().setPower(0);
//
//
//
//                driveToPosition(30, 58, 0.7, 85);
//
//                robot.getIntake().setPower(-0.5);
//                Thread.sleep(200);
//                robot.getIntake().setPower(0);
//
//
//                driveToPosition(15, 50, 1, 92);
//                driveToPosition(6, 40, 1, 92);
//
//                robot.getIntakeArmRight().setPosition(robot.intakeArmRightReset);
//                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftReset);
//
//                Thread.sleep(2000);
//
//                deposit(200, 0.5);
//                robot.getTwister().setPosition(0.9);
//
//                Thread.sleep(1000);
//
//                claw("open");
//                Thread.sleep(1000);
//
//                driveToPosition(10, 40, 0.8, 90);
//                robot.getTwister().setPosition(robot.twisterPos4);
//
//                reset(true, false);
//
//                Thread.sleep(1000);
//
//        }



    }

    @Override
    public void exit() throws InterruptedException {

    }
}
