package org.firstinspires.ftc.teamcode.Core.programs.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;


@Autonomous(name = "VisionTest", group = "OpModes")
public class VisionTest extends UpliftAutoImpl {

    public void initHardware() {

        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() {
//        robot = new UpliftRobot(this);
    }

    @Override
    public void body() throws InterruptedException {
        int location = robot.pipelineRedAudienceSide.location;

        if(location == 0 || location == -1 ) {
            robot.getFrontWebcam().stopRecordingPipeline();

            Thread.sleep(10000);

        }
    }
}
