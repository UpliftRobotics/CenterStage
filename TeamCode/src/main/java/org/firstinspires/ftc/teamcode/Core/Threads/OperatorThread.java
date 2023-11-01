package org.firstinspires.ftc.teamcode.Core.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
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

            intake();
            slides();

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

    public void intake ()
    {
        robot.getIntakeRoller().setPower(robot.opMode.gamepad2.left_stick_y);
    }
    public void slides () {
        robot.getSlideRight().setPower(robot.opMode.gamepad2.right_stick_y);
        robot.getSlideLeft().setPower(robot.opMode.gamepad2.right_stick_y);


    }
    public void liftSlides() {
        if(robot.opMode.gamepad2.a ){
            robot.getSlideRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getSlideLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.getSlideRight().setTargetPosition(500);
            robot.getSlideLeft().setTargetPosition(500);

            robot.getSlideRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.getSlideRight().setPower(.5);
            robot.getSlideRight().setPower(.5);
        }




        public void SetSlideJoyStick(){
            robot.getSlideLeft().setPower(Range.clip(robot.opMode.gamepad2.right_stick_y,
                    -1,1));
            robot.getSlideRight().setPower(Range.clip(robot.opMode.gamepad2.right_stick_y,
                    -1,1));
        }

        public void ResetOutakeTouchSensor() {
            if(robot.opMode.gamepad2.b ){
                while(!robot.getExtensionTouch().isPressed()) {
                    robot.getSlideRight().setPower(.5);
                    robot.getSlideRight().setPower(.5);}
            }
        }
    }



    public void Extension() {
        if(robot.opMode.gamepad2.a ){

            robot.getSlideRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.getSlideLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.getSlideRight().setTargetPosition(500);
            robot.getSlideLeft().setTargetPosition(500);

            robot.getSlideRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.getSlideRight().setPower(.5);
            robot.getSlideRight().setPower(.5);
        }



    }
}

}
