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
                intake();
                intakeAngle();
                slides();
                deposit();
                drop();
                rightTwister();
                leftTwister();
                reset();
                intakeDown();
                intakeup();
                closeGrabber();



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

    public void intake ()
    {
        robot.getIntakeRoller().setPower(robot.opMode.gamepad2.left_stick_y);
    }
    public void slides ()
    {
        double power = .8* -robot.opMode.gamepad2.right_stick_y;

        // if going up stop from overextending
        if (power > 0.0)
        {
            if(robot.getSlideRight().getCurrentPosition() < -850  || robot.getSlideLeft().getCurrentPosition() > 850) {
                robot.getSlideLeft().setPower(0);
                robot.getSlideRight().setPower(0);
            }
            else
            {
                robot.getSlideLeft().setPower(power);
                robot.getSlideRight().setPower(-power);
            }
        }
        // stop from overretracting
        else
        {
            if(robot.getSlideRight().getCurrentPosition() > -50  || robot.getSlideLeft().getCurrentPosition() < 50)
            {
                robot.getSlideLeft().setPower(0);
                robot.getSlideRight().setPower(0);
            }
            else
            {
                robot.getSlideLeft().setPower(power);
                robot.getSlideRight().setPower(-power);
            }
        }


        //robot.getSlideRight().setPower(.5 * robot.opMode.gamepad2.right_stick_y);
        //robot.getSlideLeft().setPower(.5 * robot.opMode.gamepad2.right_stick_y);

    }
    public void intakeAngle() throws InterruptedException {
        if (robot.opMode.gamepad2.x)
        {

            if(robot.getIntakeAngleRight().getPosition() == robot.intakeStorePos) {
                robot.getIntakeAngleRight().setPosition(robot.intakeGroundPos);}
            else if (robot.getIntakeAngleRight().getPosition() == robot.intakeGroundPos) {
                robot.getIntakeAngleRight().setPosition(robot.intake2Pixel);}
            else if (robot.getIntakeAngleRight().getPosition() == robot.intake2Pixel) {
                robot.getIntakeAngleRight().setPosition(robot.intake3Pixel);}
            else if (robot.getIntakeAngleRight().getPosition() == robot.intake3Pixel) {
                robot.getIntakeAngleRight().setPosition(robot.intake4Pixel);}
            else if (robot.getIntakeAngleRight().getPosition() == robot.intake4Pixel) {
                robot.getIntakeAngleRight().setPosition(robot.intake5Pixel);}
            else
                robot.getIntakeAngleRight().setPosition(robot.intakeStorePos);
        }
    }

    public void deposit () throws InterruptedException {
        if(robot.opMode.gamepad2.y )
        {
            if (robot.depositStage == 0)
            {
                robot.getGrabber().setPosition(robot.grabberOpen);
                robot.getDepositWrist().setPosition(robot.wristPick);
                Thread.sleep(400);
                robot.getDepositArm().setPosition(robot.depositPick);
                Thread.sleep(400);
                robot.getDepositWrist().setPosition(robot.wristPick2);
                Thread.sleep(400);
                robot.getDepositArm().setPosition(robot.depositPick2);
                Thread.sleep(400);
                robot.getIntakeAngleRight().setPosition(robot.intake4Pixel);
                Thread.sleep(500);
                robot.getGrabber().setPosition(robot.grabberClose);
                Thread.sleep(2000);

                robot.getDepositArm().setPosition(robot.depositHold);
                robot.getDepositWrist().setPosition(robot.wristPick);
                robot.depositStage++;
            }
            else if (robot.depositStage == 1)
            {

                robot.getDepositWrist().setPosition(robot.wristBack);
                robot.getDepositArm().setPosition(.5);
                Thread.sleep(500);
                robot.getDepositArm().setPosition(.6);
                Thread.sleep(500);
                robot.getDepositArm().setPosition(.7);
                Thread.sleep(500);
                robot.getDepositArm().setPosition(.8);
                Thread.sleep(1000);
                robot.getDepositArm().setPosition(robot.depositBack);
                robot.depositStage++;
            }
        }

    }
    public void drop () throws InterruptedException {
        if (robot.opMode.gamepad2.right_trigger > .5 && robot.depositStage == 2)
        {
            robot.getGrabber().setPosition(robot.grabberOpen);
//            sleep(500);
//            robot.getDepositArm().setPosition(robot.depositHold);
//            robot.getDepositWrist().setPosition(robot.wristHold);
//            robot.getDepositTwist().setPosition(robot.twistReset);
//            robot.getGrabber().setPosition(robot.grabberClose);
//            robot.depositStage = 0;
//            robot.slidesDown()
        }
    }
    public void rightTwister() throws InterruptedException {
        if (robot.opMode.gamepad2.right_bumper && robot.depositStage == 2)
        {
            double twisterPos = robot.getDepositTwist().getPosition();
             if(twisterPos + robot.twistPosIncremnt > 1 )
             {
                robot.getDepositTwist().setPosition(twisterPos - ( 3 * robot.twistPosIncremnt));
                sleep(600);
             }
             else
             {
                 robot.getDepositTwist().setPosition(twisterPos + robot.twistPosIncremnt);
                 sleep(200);
             }

        }

    }
    public void leftTwister() throws InterruptedException {
        if (robot.opMode.gamepad2.right_bumper && robot.depositStage == 2)
        {
            double twisterPos = robot.getDepositTwist().getPosition();
            if(twisterPos - robot.twistPosIncremnt < 0 )
            {
                robot.getDepositTwist().setPosition(twisterPos + ( 3 * robot.twistPosIncremnt));
                sleep(600);
            }
            else
            {
                robot.getDepositTwist().setPosition(twisterPos - robot.twistPosIncremnt);
                sleep(200);
            }

        }

    }
    public void reset ()
    {
        if ( robot.opMode.gamepad2.dpad_down)
        {
            robot.depositStage = 0;
            robot.getIntakeAngleRight().setPosition(robot.intakeStorePos);

//            robot.getSlideRight().setTargetPosition(0);
//            robot.getSlideLeft().setTargetPosition(0);
//
//            robot.getSlideRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.getSlideRight().setPower(-.5);
//            robot.getSlideRight().setPower(-.5);

            robot.getDepositArm().setPosition(robot.depositHold);
            robot.getDepositWrist().setPosition(robot.wristHold);
            robot.getDepositTwist().setPosition(robot.twistReset);


        }

    }
    public void hang()
    {
//        if (robot.opMode.gamepad2.dpad_left && robot.opMode.gamepad2.left_trigger > .5)
//        {
//            robot.getHang1().setPosition(robot.hangOpenPos);
//            robot.getHang2().setPosition(robot.hangOpenPos);
//        }
    }
    public void plane()
    {
//        if(robot.opMode.gamepad2.dpad_right && robot.opMode.gamepad2.left_trigger >.5)
//        {
//            robot.getPlane().setPosition(robot.planeFire);
//        }
    }

//    public void liftSlides() {
//        if(robot.opMode.gamepad2.a ){
//            robot.getSlideRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.getSlideLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.getSlideRight().setTargetPosition(500);
//            robot.getSlideLeft().setTargetPosition(500);
//
//            robot.getSlideRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.getSlideRight().setPower(.5);
//            robot.getSlideRight().setPower(.5);
//        }




//        public void SetSlideJoyStick()
//    {
//            robot.getSlideLeft().setPower(Range.clip(robot.opMode.gamepad2.right_stick_y, -1,1));
//            robot.getSlideRight().setPower(Range.clip(robot.opMode.gamepad2.right_stick_y, -1,1));
//        }

//        public void ResetOutakeTouchSensor() {
//            if(robot.opMode.gamepad2.b ){
//                while(!robot.getExtensionTouch().isPressed())
//                {
//                    robot.getSlideRight().setPower(.5);
//                    robot.getSlideRight().setPower(.5);
//                }
//            }
//        }
//    }



//    public void Extension() {
//        if(robot.opMode.gamepad2.a )
//        {
//            robot.getSlideRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.getSlideLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.getSlideRight().setTargetPosition(500);
//            robot.getSlideLeft().setTargetPosition(500);
//
//            robot.getSlideRight().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.getSlideLeft().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.getSlideRight().setPower(.5);
//            robot.getSlideRight().setPower(.5);
//        }
    public void intakeDown()
    {
        if(robot.opMode.gamepad2.dpad_left)
        {
            robot.getIntakeAngleRight().setPosition(robot.intakeGroundPos);
        }
    }

    public void intakeup()
    {
        if(robot.opMode.gamepad2.dpad_right)
        {
            robot.getIntakeAngleRight().setPosition(robot.intake4Pixel);
        }
    }

    public void closeGrabber()
    {
        if(robot.opMode.gamepad2.dpad_down)
        {
            robot.getGrabber().setPosition(robot.grabberClose);
        }
    }





//    }
}

//}
