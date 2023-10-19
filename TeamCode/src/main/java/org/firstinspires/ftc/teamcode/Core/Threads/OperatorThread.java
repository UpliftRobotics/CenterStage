package org.firstinspires.ftc.teamcode.Core.Threads;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;

public class OperatorThread extends Thread
{


    private UpliftRobot robot;

    private static final String OPERATOR_NAME = "OperatorThreadName";

    private boolean shutDown = false;



    public OperatorThread(UpliftRobot robot)
    {
        this.robot = robot;

    }

    @Override
    public void run()
    {
        while(!shutDown)
        {
            try
            {




//                robot.opMode.telemetry.addData("magnet", robot.getMagnet().isPressed());
//                robot.opMode.telemetry.addData("odoRight" , robot.getOdoRight().getCurrentPosition());
//                robot.opMode.telemetry.update();




                // todo: validate user responsiveness and set sleep
//                sleep(50);
            } catch (Exception e) {
                e.printStackTrace();

//                StringWriter sw = new StringWriter();
//                PrintWriter pw = new PrintWriter(sw);
//                e.printStackTrace(pw);
//
//                robot.opMode.telemetry.addData("Operator error ", e.getMessage());
//                robot.opMode.telemetry.addData("Operator error stack", sw.toString());
            }
        }
    }

    public void end()
    {
        shutDown = true;

        robot.opMode.telemetry.addData("Operator Thread stopped ", shutDown);

        robot.opMode.telemetry.update();
    }

    @Override
    public String toString()
    {
        return "OperatorThread{" +
                "name=" + OPERATOR_NAME +
                '}';
    }
}
