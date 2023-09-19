package org.firstinspires.ftc.teamcode.Core.Threads;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;

public class OperatorThread extends Thread
{


    private UpliftRobot robot;

    private static final String OPERATOR_NAME = "OperatorThreadName";

    private boolean shutDown = false;

    private boolean grabberState;
    private boolean blockGrabberInput;

    private boolean fourBarState;
    private boolean block4BarInput;



    private boolean twisterState;
    private boolean blockTwisterInput;



    public OperatorThread(UpliftRobot robot)
    {
        this.robot = robot;

        this.grabberState = true;
        this.blockGrabberInput = false;

        this.fourBarState = true;
        this.block4BarInput = false;

    }

    @Override
    public void run()
    {
        while(!shutDown)
        {
            try
            {



//                toggleGrab();

                toggle4Bar();

                reset();

                servoArmsHigh();

                highBackwards();

                highForwards();

                mediumBackwards();

                coneRighting();

                stackHeight5();

                stackHeight4();

                stackHeight3();

                stackHeight2();

                holdSlidePos();

//                senseCone();

                if(robot.opMode.gamepad2.left_bumper)
                {
                    robot.getFourBar().setPosition(robot.getFourBar().getPosition() + 0.1);
                }

                if(robot.opMode.gamepad2.right_bumper)
                {
                    robot.getFourBar().setPosition(.75);
                }

//                robot.getSlide1().setPower(0.85 * Range.clip(robot.opMode.gamepad2.right_stick_y, -1, 1));
//                robot.getSlide2().setPower(0.85 * Range.clip(-robot.opMode.gamepad2.right_stick_y, -1, 1));
                robot.getSlide1().setPower(Range.clip(robot.opMode.gamepad2.right_stick_y, -1, 1));
                robot.getSlide2().setPower(Range.clip(-robot.opMode.gamepad2.right_stick_y, -1, 1));





//                if(robot.getMagnet().isPressed())
//                {
////                    robot.getArmLeft().setPosition(robot.getArm1HighPos());
////                    robot.getArmRight().setPosition(robot.getArm2HighPos());
//                }
//                if(robot.getMagnet().isPressed())
//                {
//                    robot.getArmLeft().setPosition(robot.getArm1HighPos());
//                    robot.getArmRight().setPosition(robot.getArm2HighPos());
//                }


                if (robot.opMode.gamepad2.right_trigger > .1)
                {

                    robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
                }

                if (robot.opMode.gamepad2.b)
                {
                    robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
                }



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


//    public void slides(double power, int dist)
//    {
//
//        double initialPos2 = robot.getSlide2().getCurrentPosition();
//
//        while (Math.abs(robot.getSlide2().getCurrentPosition() - (initialPos2 + dist)) > 50)
//        {
//            robot.getSlide1().setPower(-0.9 * power);
//            robot.getSlide2().setPower(power);
//        }
//        robot.getSlide1().setPower(0);
//        robot.getSlide2().setPower(0);
//    }


    public void toggle4Bar()
    {
        //allows the 4bar to move to its forwards and backwards position with the same button
        if(robot.opMode.gamepad2.x && !block4BarInput)
        {
//            robot.getFourBar().setPosition(fourBarState ? 0.85 : .5);
//            robot.getFourBar2().setPosition(fourBarState ? robot.getBar2FrontPos() : 0.62);
            robot.getFourBar().setPosition(fourBarState ? 0.25 : .75);
            fourBarState = !fourBarState;
            block4BarInput = true;
           // while(robot.opMode.gamepad2.x && robot.getArmLeftLowPos().)
        }
        else if(!robot.opMode.gamepad2.x && block4BarInput)
        {
            block4BarInput = false;
        }
    }

//    public void toggleGrab() throws InterruptedException
//    {
//        if(robot.opMode.gamepad2.right_trigger > robot.getGrabber1OpenPos() && !blockGrabberInput)
//        {
//        robot.getGrabber1().setPosition(grabberState ? robot.getGrabber1OpenPos() : robot.getGrabber1ClosePos());
//        grabberState = !grabberState;
//        blockGrabberInput = true;
//        }
//        else if (robot.opMode.gamepad2.right_trigger < robot.getGrabber1ClosePos() && blockGrabberInput)
//        {
//            blockGrabberInput = false;
//        }
//    }

    public void senseCone()
    {

        double value = robot.getConeDetector().getDistance(DistanceUnit.CM);
        if(value <= 5 && robot.opMode.gamepad2.right_trigger == 0)
        {
            robot.getGrabber1().setPosition(robot.getGrabber1ClosePos());
        }
        else
        {
            if(robot.opMode.gamepad2.right_trigger > robot.getGrabber1OpenPos() && !blockGrabberInput)
            {
                robot.getGrabber1().setPosition(grabberState ? robot.getGrabber1OpenPos() : robot.getGrabber1ClosePos());
                grabberState = !grabberState;
                blockGrabberInput = true;
            }
            else if (robot.opMode.gamepad2.right_trigger < robot.getGrabber1ClosePos() && blockGrabberInput)
            {
                blockGrabberInput = false;
            }
        }
    }

    public void reset() throws InterruptedException {
        if (robot.opMode.gamepad2.a) // reset grabber pos
        {
            //   while(!robot.getMagnet().isPressed())
//            {
//                robot.getSlide1().setPower(0.5);
//                robot.getSlide2().setPower(-0.5);
//            }

            robot.getSlide1().setPower(0);
            robot.getSlide2().setPower(0);
            //move 6 bar motors down
//            while(!robot.getMagnet().isPressed())
//            {
//                robot.getSlide1().setPower(0.2);
//                robot.getSlide2().setPower(-0.2);
//            }
            robot.getTwister().setPosition(robot.getTwisterDownPos());
//            robot.getFourBar1().setPosition(0.8);
//            robot.getFourBar2().setPosition(0.2);
            robot.getFourBar().setPosition(robot.getBarFrontPos()+ .03);

//            Thread.sleep(500);

            robot.getArmLeft().setPosition(robot.getArmLeftLowPos());
            robot.getArmRight().setPosition(robot.getArmRightLowPos());
//            Thread.sleep(500);



        }
    }


    public void servoArmsHigh()
    {
        if (robot.opMode.gamepad2.y) // raise servo on 6bar
        {
            robot.getArmRight().setPosition(.05);
            robot.getArmLeft().setPosition(.95);
//            robot.getArmRight().setPosition(.05);
//            robot.getArmLeft().setPosition(.95);
           robot.getFourBar().setPosition(.3);
//
//            robot.getTwister().setPosition(robot.getTwisterUpPos());
        }

    }

    public void highBackwards()
    {
        if(robot.opMode.gamepad2.dpad_up) // max height backwards
        {

//            slides(0.5, 953);

            robot.getFourBar().setPosition(robot.getBarBackPos());

            robot.getTwister().setPosition(robot.getTwisterUpPos());
//            robot.getArmLeft().setPosition(robot.getArmLeftHighPos());
//            robot.getArmRight().setPosition(robot.getArmRightHighPos());

//            servoArmsHigh();
            robot.getArmRight().setPosition(.05);
            robot.getArmLeft().setPosition(.95);
        }
    }

    public void highForwards()
    {
        if(robot.opMode.gamepad2.dpad_left) // max height forwards
        {
//            slides(0.5, 953);
//            robot.getArmLeft().setPosition(robot.getArm1HighPos());
//            robot.getArmRight().setPosition(robot.getArm2HighPos());

            robot.getFourBar().setPosition(robot.getBarFrontPos());

            robot.getTwister().setPosition(robot.getTwisterDownPos());

//            armHigh();

//            slidesDist(0.5, 953);


        }
    }

    public void mediumBackwards()
    {
        if(robot.opMode.gamepad2.dpad_right) //  medium height backwards
        {
//            robot.getTwister().setPosition(robot.getTwisterUpPos());


            robot.getTwister().setPosition(robot.getTwisterUpPos());

//            armHigh();

//            slidesDist(0.5, 500);
            robot.getFourBar().setPosition(0.5);
////            slides(0.5, 700);
//            robot.getArm1().setPosition(robot.getArm1HighPos());
//            robot.getArm2().setPosition(robot.getArm2HighPos());
//
//            robot.getFourBar1().setPosition(robot.getBar1BackPos());
//            robot.getFourBar2().setPosition(robot.getBar2BackPos());
//
//            robot.getTwister().setPosition(robot.getTwisterUpPos());
//
//            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
//            robot.getFourBar2().setPosition(robot.getBar2FrontPos());

        }
    }

    public void coneRighting()
    {
        if(robot.opMode.gamepad2.dpad_down) // medium height forwards
        {


            robot.getTwister().setPosition(robot.getTwisterDownPos());
            robot.getArmLeft().setPosition(robot.getArm1StackPos5());
            robot.getArmRight().setPosition(robot.getArm2StackPos5());

            robot.getFourBar().setPosition(.05);

            robot.getGrabber1().setPosition(robot.getGrabber1OpenPos());
////            slides(0.5, 700);
//            robot.getArm1().setPosition(robot.getArm1HighPos());
//            robot.getArm2().setPosition(robot.getArm2HighPos());
//
//            robot.getFourBar1().setPosition(robot.getBar1FrontPos());
//            robot.getFourBar2().setPosition(robot.getBar2FrontPos());
//
//            robot.getTwister().setPosition(robot.getTwisterDownPos());
//
//            robot.getFourBar1().setPosition(.38);
//            robot.getFourBar2().setPosition(.62);





        }
    }

    public void stackHeight5()
    {
        if(robot.opMode.gamepad2.left_stick_y > .5) // 5 stack height
        {
            robot.getArmLeft().setPosition(robot.getArm1StackPos5());
            robot.getArmRight().setPosition(robot.getArm2StackPos5());

        }
    }

    public void stackHeight4()
    {
        if(robot.opMode.gamepad2.left_stick_x < -0.5) // 4 stack height
        {
            robot.getArmLeft().setPosition(robot.getArm1StackPos4());
            robot.getArmRight().setPosition(robot.getArm2StackPos4());
        }
    }

    public void stackHeight3()
    {
        if(robot.opMode.gamepad2.left_stick_x > 0.5) // 3 stack height
        {

            robot.getArmLeft().setPosition(robot.getArm1StackPos3());
            robot.getArmRight().setPosition(robot.getArm2StackPos3());
        }
    }

    public void stackHeight2()
    {
        if(robot.opMode.gamepad2.left_stick_y < -0.5) // 2 stack height
        {
            robot.getArmLeft().setPosition(robot.getArm1StackPos2()-.01);
            robot.getArmRight().setPosition(robot.getArm2StackPos2()+.01);
        }
    }



    public void holdSlidePos()
    {
        if(robot.opMode.gamepad2.left_trigger > 0)
        {
            robot.getSlide1().setPower(-0.2);
            robot.getSlide2().setPower(0.2);
        }
    }

    public void hold()
    {
        if(robot.opMode.gamepad2.left_trigger > 0)
        {
            robot.getSlide1().setPower(-0.2);
            robot.getSlide2().setPower(0.2);
        }
    }

    @Override
    public String toString()
    {
        return "OperatorThread{" +
                "name=" + OPERATOR_NAME +
                '}';
    }
}
