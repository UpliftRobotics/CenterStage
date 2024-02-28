package org.firstinspires.ftc.teamcode.Core.toolkit.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedCloseBlueFarWhite extends OpenCvPipeline {

    public boolean redClose = false;

    Telemetry telemetry;
    Mat mat = new Mat();
    public int location = -1;

    static final Rect MIDDLE_ROI = new Rect(
            new Point(60, 80),
            new Point(140, 125));

    static final Rect RIGHT_ROI = new Rect(
            new Point(200, 80),
            new Point(260, 140));

    public RedCloseBlueFarWhite(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        System.out.println("Thread2: " + Thread.currentThread().getName());
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 0, 0);
        Scalar highHSV = new Scalar(0, 0, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        //Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        //rectangle(image, rect, Scalar(0,255,0), 1, 8, 0);

        //double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        // left.release();
        middle.release();
        right.release();

//        //telemetry.addData("Left Raw Value", (int) Core.sumElems(left).val[0]);
//        telemetry.addData("Middle Raw Value", (int) Core.sumElems(middle).val[0]);
//         telemetry.addData("Right Raw Value", (int) Core.sumElems(right).val[0]);
//        //telemetry.addData("Left Percentage", Math.round(leftValue * 100) + "%");
//        telemetry.addData("Middle Percentage", Math.round(middleValue * 100) + "%");
//        telemetry.addData("Right Percentage", Math.round(rightValue * 100) + "%");
//
//       // Imgproc.rectangle(input, LEFT_ROI, new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(input, MIDDLE_ROI, new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(input, RIGHT_ROI, new Scalar(0, 255, 0), 4);
//        //telemetry.addData("left", leftValue);
//        telemetry.addData("middle", middleValue);
//        telemetry.addData("right", rightValue);
        telemetry.addData("location", location);

        telemetry.addData("redClose: ", redClose);

        telemetry.update();


        if(0.2 > middleValue&&  0.2 > rightValue) {
            location = 0;
            redClose = true;
        }

        if(middleValue > .2 && middleValue > rightValue) {
            location = 1;
            redClose = true;
        }

        else if(rightValue > middleValue && rightValue > 0.2) {
            location = 2;
            redClose = true;

        }

        return input;

    }
}

