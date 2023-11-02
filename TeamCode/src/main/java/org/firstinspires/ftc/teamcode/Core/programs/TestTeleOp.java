package org.firstinspires.ftc.teamcode.Core.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Threads.DriveThread;
import org.firstinspires.ftc.teamcode.Core.Threads.OperatorThread;
import org.firstinspires.ftc.teamcode.Core.main.UpliftRobot;
import org.firstinspires.ftc.teamcode.Core.main.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.toolkit.Odometry;


@TeleOp(name = "TestTeleOp", group = "Opmodes")
public class TestTeleOp extends UpliftTele {

        UpliftRobot robot;

        Odometry odom;


        DriveThread driverThread;
        OperatorThread operatorThread;

        @Override
        public void initHardware() {
            robot = new UpliftRobot(this);
            odom = robot.odometry;

            createDriveThread(robot);
            createOperatorThread(robot);
        }

        @Override
        public void initAction() {
                driverThread.start();
                operatorThread.start();
                odom.setOdometryPosition(0, 0, 0);
        }

        @Override

        public void bodyLoop() throws InterruptedException {

        }

        @Override
        public void exit()
        {
                driverThread.end();
                operatorThread.end();
        }
        public void createDriveThread(UpliftRobot robot1)
        {

                driverThread = new DriveThread(robot1);
                telemetry.addData("Driver Thread started", driverThread.toString());

        }

        public void createOperatorThread(UpliftRobot robot1) {

                operatorThread = new OperatorThread(robot1);
                telemetry.addData("Operator Thread started", operatorThread.toString());

        }

}
