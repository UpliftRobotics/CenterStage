package org.firstinspires.ftc.teamcode.Core.programs.Autos.NoCycles;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "Blue Deposit 2 + 0", group = "Opmodes")
public class BlueDeposit2Plus0 extends UpliftAutoImpl
{
    Odometry odom;


    public void initHardware() {

        robot = new UpliftRobot(this);
        odom = robot.odometry;

    }

    @Override
    public void initAction() throws InterruptedException
    {

        robot.getExtension().setPower(.05);
        claw("open");

        robot.getIntake().setPower(.2);

        robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftTransfer);
        robot.getIntakeArmRight().setPosition(robot.intakeArmRightTransfer);
        robot.getIntakeRoller().setPosition(robot.frontRollerStore);


        robot.getTwister().setPosition(robot.twisterPos4);
        robot.getArmRight().setPosition(robot.armRightStore);
        robot.getArmLeft().setPosition(robot.armLeftStore);
        robot.getDepositWrist().setPosition(robot.depositWristTransfer1);

        Thread.sleep(3000);

        robot.getDepositWrist().setPosition(robot.depositWristTransfer2);
        robot.getArmRight().setPosition(robot.armRightTransfer);
        robot.getArmLeft().setPosition(robot.armLeftTransfer);

        Thread.sleep(500);

        claw("close1");
        robot.getIntake().setPower(0);

        Thread.sleep(1000);



        robot.frontWebcam.setPipeline(robot.pipelineBlueDepositSide);

    }

    @Override
    public void body() throws InterruptedException
    {
//        robot.getExtension().setPower(.05);
//        int location = robot.pipelineBlueDepositSide.location;
        int location = 0;
        odom.setOdometryPosition(48, 0, 0);

        if(!goPark)
        {
            //left
            if(location == 0 || location == -1 )
            {
                //drop purple pixel position
                driveToPosition(42, 28, 0.9, 0);

//                drop yellow pixel position
                driveToPosition(42, 15, 0.9, 0);
                driveToPosition(15, 24, 0.5, 85);

                robot.frontWebcam.closeCameraDevice();

                driveToAprilTag(14.8, 24, 85);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);


                deposit(0, 0.5);
                Thread.sleep(500);
                claw("open");
                reset(true, false);

                odom.setOdometryPosition(12, 24, 85);
                driveToPosition(15, 5, 0.9, 85);
                Thread.sleep(1000);


            }

            //middle
            if(location == 1 )
            {
                //drop purple pixel position
                driveToPosition(48, 31, 0.8, 0);

                //drop yellow pixel position
                driveToPosition(48, 19, 0.8, 0);
                driveToPosition(15, 32, 0.5, 85);

                robot.frontWebcam.closeCameraDevice();

                driveToAprilTag(14.75, 32, 85);

                robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
                robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);


                deposit(0, 0.5);

                Thread.sleep(500);
                claw("open");
                reset(true, false);

                odom.setOdometryPosition(12, 32, 85);
                driveToPosition(15, 8, 0.9, 85);
                Thread.sleep(1000);


            }

            // right

            if(location == 2 )
            {
                //drop purple pixel position
                driveToPosition(48, 20, 0.6, 0);
                driveToPosition(56, 28, 0.6, 50);


                //drop yellow pixel position
                driveToPosition(42, 14, 0.9, 50);
                driveToPosition(15, 41, 0.6, 75);

            robot.frontWebcam.closeCameraDevice();

            driveToAprilTag(13, 41, 75);

            robot.getIntakeArmRight().setPosition(robot.intakeArmRightStack3);
            robot.getIntakeArmLeft().setPosition(robot.intakeArmLeftStack3);


            deposit(0, 0.5);

            Thread.sleep(500);
            claw("open");
            reset(true, false);

            odom.setOdometryPosition(12, 41, 75);
            driveToPosition(15, 8, 0.9, 75);
            Thread.sleep(1000);

            }

//            driveToPosition(20, 32, 0.6, 90);
//            driveToPosition(5, 10, 0.6, 90);
        }






    }

    @Override
    public void exit() throws InterruptedException {

    }
}
