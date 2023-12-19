package org.firstinspires.ftc.teamcode.Core.toolkit.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStageBlueFar extends OpenCvPipeline {

    public boolean blueFar = false;

    Telemetry telemetry;
    Mat mat = new Mat();
    public int location = -1;
//    static final Rect LEFT_ROI = new Rect(
//            new Point(0, 100),
//            new Point(60, 170));

    static final Rect RIGHT_ROI = new Rect(
            new Point(220, 100),
            new Point(320, 170));

    static final Rect MIDDLE_ROI = new Rect(
            new Point(80, 100),
            new Point(160,170));

    public CenterStageBlueFar(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        System.out.println("Thread2: " + Thread.currentThread().getName());
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(90, 50, 70);
        Scalar highHSV = new Scalar(128, 255, 255);


        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        // Mat right = mat.submat(RIGHT_ROI);

        //rectangle(image, rect, Scalar(0,255,0), 1, 8, 0);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        //double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        right.release();
        middle.release();
        //right.release();

        telemetry.addData("Left Raw Value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle Raw Value", (int) Core.sumElems(middle).val[0]);
        // telemetry.addData("Right Raw Value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left Percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle Percentage", Math.round(middleValue * 100) + "%");
        //telemetry.addData("Right Percentage", Math.round(rightValue * 100) + "%");

        Imgproc.rectangle(input, RIGHT_ROI, new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(input, MIDDLE_ROI, new Scalar(0, 255, 0), 4);
        //Imgproc.rectangle(input, RIGHT_ROI, new Scalar(0, 255, 0), 4);
        telemetry.addData("left", rightValue);
        telemetry.addData("middle", middleValue);
        //telemetry.addData("right", rightValue);
        telemetry.addData("location", location);
        telemetry.update();


        if(rightValue > middleValue && rightValue > 0.2) {
            location = 2;
            blueFar = true;
        }

        if(middleValue > .2 && middleValue > rightValue) {
            location = 1;
            blueFar = true;
        }

        else if(0.2 > middleValue && 0.2 > rightValue) {
            location = 0;
            blueFar = true;
        }

        return input;

    }
}
