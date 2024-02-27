package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;


@TeleOp(name="Test April Tags")
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor aprilProcessor = new AprilTagProcessor.Builder()

                .setLensIntrinsics(822.317, 822.317, 319.495, 242.05)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(aprilProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while(!isStopRequested() && opModeIsActive())
        {

            if(aprilProcessor.getDetections().size() > 0)
            {
                AprilTagDetection tag = aprilProcessor.getDetections().get(0);
                telemetry.addData("Tag", tag.id);
                telemetry.addData("X: ", tag.ftcPose.x);
                telemetry.addData("Y: ", tag.ftcPose.y);
                telemetry.addData("Angle", tag.ftcPose.yaw);
                telemetry.update();
            }


            if (gamepad1.dpad_down)
            {

            }
        }
    }
}
