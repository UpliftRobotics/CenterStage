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

        odom.setOdometryPosition(0,0,0);
//        driveToPosition(24,108,.5,90);
        robot.frontWebcam.closeCameraDevice();

//        driveToAprilTag(0, 0, 0);

        newAprilTagMethod(0.1);

//        Thread.sleep(20000);






    }

    @Override
    public void exit() throws InterruptedException {

    }
}
