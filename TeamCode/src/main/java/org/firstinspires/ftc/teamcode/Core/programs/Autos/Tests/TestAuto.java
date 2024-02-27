package org.firstinspires.ftc.teamcode.Core.programs.Autos.Tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "TestAuto", group = "Opmodes")
public class TestAuto extends UpliftAutoImpl
{

    Odometry odom;

    public void initHardware()
    {

        robot = new UpliftRobot(this);
        odom = robot.odometry;


    }

    @Override
    public void initAction() throws InterruptedException
    {

    }

    @Override
    public void body() throws InterruptedException
    {
//        robot.frontWebcam.closeCameraDevice();
//
//        AprilTagProcessor aprilProcessor = new AprilTagProcessor.Builder()
//
//                .setLensIntrinsics(822.317, 822.317, 319.495, 242.05)
//                .build();
//
//        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(aprilProcessor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .setCameraResolution(new Size(640, 480))
//                .build();
//        odom.setOdometryPosition(30,108,90);
//
//        driveToAprilTag(24,108,90);
//        if(aprilProcessor.getDetections().size() > 0)
//        {
//            AprilTagDetection tag = aprilProcessor.getDetections().get(0);
//            if(Math.abs(tag.ftcPose.x) > .5 || ((tag.ftcPose.y < 15 || tag.ftcPose.y > 16)) || Math.abs(tag.ftcPose.yaw) > 2)
//            {
//                odom.setOdometryPosition(0,0,90);
//                driveToPosition(-tag.ftcPose.y+15,tag.ftcPose.x,.5,90 + tag.ftcPose.yaw);
//            }
//        }
//
//        Thread.sleep(1000);

        odom.setOdometryPosition(30,108,90);
        driveToPosition(24,108,.5,90);
        robot.frontWebcam.closeCameraDevice();

        SlowdriveToAprilTag(24,108,90);


        Thread.sleep(20000);






    }

    @Override
    public void exit() throws InterruptedException {

    }
}
