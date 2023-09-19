package org.firstinspires.ftc.teamcode.Core.toolkit.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class  PowerPlay extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public int location;
    static final Rect ROI = new Rect(
            new Point(400, 50),
            new Point(550, 200));

    public PowerPlay(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        System.out.println("Thread2: " + Thread.currentThread().getName());
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat box = mat.submat(ROI);

        Scalar purplelowHSV = new Scalar(110, 80, 60);
        Scalar purplehighHSV = new Scalar(140, 255, 255);

        Scalar greenlowHSV = new Scalar(30, 80, 60);
        Scalar greenhighHSV = new Scalar(110, 255, 255);

        Scalar orangelowHSV = new Scalar(10, 80, 60);
        Scalar orangehighHSV = new Scalar(30, 255, 255);

        Mat purpleMat = new Mat();
        Mat greenMat = new Mat();
        Mat orangeMat = new Mat();

        Core.inRange(box, orangelowHSV, orangehighHSV, orangeMat);
        Core.inRange(box, greenlowHSV, greenhighHSV, greenMat);
        Core.inRange(box, purplelowHSV, purplehighHSV, purpleMat);





        //rectangle(image, rect, Scalar(0,255,0), 1, 8, 0);

        double OrangeValue = (int) Core.sumElems(orangeMat).val[0];
        double PurpleValue = (int) Core.sumElems(purpleMat).val[0];
        double GreenValue = (int) Core.sumElems(greenMat).val[0];
//
//        box.release();
//
//
//        telemetry.addData("Orange Raw Value", OrangeValue);
//
//        telemetry.addData("Orange Percentage", Math.round(OrangeValue * 100) + "%");
//
//        telemetry.addData("Purple Raw Value", (int) PurpleValue);
//
//        telemetry.addData("Purple Percentage", Math.round(PurpleValue * 100) + "%");
//
//        telemetry.addData("Green Raw Value", (int) GreenValue);
//
//        telemetry.addData("Green Percentage", Math.round(GreenValue * 100) + "%");

        Imgproc.rectangle(input, ROI, new Scalar(0, 255, 0), 4);

//
//
        if (GreenValue > OrangeValue && GreenValue > PurpleValue)
        {
            location = 1;
        } else if (OrangeValue > PurpleValue && OrangeValue > GreenValue)
        {
            location = 3;
        } else
        {
            location = 2;
        }
        telemetry.addData("Location:", location);
        telemetry.update();
        return input  ;
    }

    }



