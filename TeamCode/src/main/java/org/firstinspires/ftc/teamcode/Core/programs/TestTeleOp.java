package org.firstinspires.ftc.teamcode.Core.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;


@TeleOp(name = "TestTeleOp", group = "Opmodes")
public class TestTeleOp extends UpliftTele {

        UpliftRobot robot;

        @Override
        public void initHardware() {
            robot = new UpliftRobot(this);
        }

        @Override
        public void initAction() {

        }

        @Override

        public void bodyLoop() throws InterruptedException {

            telemetry.addData("Right Joystick Y Val: ", gamepad2.right_stick_y);
            telemetry.update();

        }

        @Override
        public void exit()
        {

        }

}
