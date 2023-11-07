package org.firstinspires.ftc.teamcode.Core.programs.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import static org.firstinspires.ftc.teamcode.Core.main.UpliftRobot.QUICKEST_DIRECTION;

import org.firstinspires.ftc.teamcode.Core.main.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;



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
        int location = robot.pipeline.location;

        if(location == 0 || location == -1 ) {
            robot.getWebcam().stopRecordingPipeline();

            Thread.sleep(10000);

        }
          }
}
