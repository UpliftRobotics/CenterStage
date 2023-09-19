package org.firstinspires.ftc.teamcode.Core.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class UpliftAutoImpl extends UpliftAuto {

    public UpliftRobot robot;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);

    }

    @Override
    public void initAction() {

    }


    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() throws InterruptedException {

    }


}
