package org.firstinspires.ftc.teamcode.Core.toolkit.vision;

import static org.opencv.core.Core.inRange;

import android.os.Build;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

;

public class ConesPipeline extends OpenCvPipeline {


    /*
   --------------------------------------------------------------------------------------------------------
   VISION FOR CONES
   --------------------------------------------------------------------------------------------------------
   */
    private final Mat labCones = new Mat();

    private final Size kSize = new Size(5, 5);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kSize);

    // Cone mask scalars
    private final Scalar redLow = new Scalar(0, 161, 60);
    private final Scalar redHigh = new Scalar(235, 255, 255);
    private final Scalar blueLow = new Scalar(0, 100, 155);
    private final Scalar blueHigh = new Scalar(235, 255, 255);

    // Mat objects
    private final Mat maskRed = new Mat();
    private final Mat erodeRed = new Mat();
    private final Mat dilateRed = new Mat();
    private final Mat maskBlue = new Mat();
    private final Mat erodeBlue = new Mat();
    private final Mat dilateBlue = new Mat();

    private final List<MatOfPoint> redContours = new ArrayList<>();
    private final List<MatOfPoint> blueContours = new ArrayList<>();

    private Rect redRect = new Rect();
    private Rect blueRect = new Rect();

    public double horizon_cones = 0;
    private final Scalar HORIZON_COLOR_CONES = new Scalar(0,255,0);
    private final Scalar CONTOUR_COLOR_CONES = new Scalar(255,0,255);

    public double CONTOUR_AREA_CONES = 250.0;

    private final Scalar TEXT_COLOR_CONES = new Scalar(0, 0, 0);

    private double error = 0.0;

    // RED OR NOT (BLUE)
    public boolean detectRed = false;

    private boolean savePictureFlag = false;

    @Override
    public Mat processFrame(Mat source0) {

        Mat submat = source0.submat(new Rect (0,0,800,200));

        Imgproc.cvtColor(submat, labCones, Imgproc.COLOR_RGB2YCrCb);

        Mat lowestMat = new Mat();
        int prevYPos = Integer.MAX_VALUE;
        if (detectRed) {
            inRange(labCones, redLow, redHigh, maskRed);
            Imgproc.erode(maskRed, erodeRed, kernel );
            Imgproc.dilate(erodeRed, dilateRed, kernel);

            redContours.clear();

            Imgproc.findContours(dilateRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                redContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon_cones
                        || (Imgproc.boundingRect(c).y > 400 && (Imgproc.boundingRect(c).height)/(Imgproc.boundingRect(c).width) < 1));
            }
            Imgproc.drawContours(submat, redContours, -1, CONTOUR_COLOR_CONES);

            if(!redContours.isEmpty()) {
                MatOfPoint biggestRedContour = null;
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                    biggestRedContour = Collections.max(redContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                }
                if(Imgproc.contourArea(biggestRedContour) > CONTOUR_AREA_CONES) {

                    redRect = Imgproc.boundingRect(biggestRedContour);
                    error = 400 - (redRect.width / 2 + redRect.x);

//
                    Log.i("height/width: ", "" + redRect.height/redRect.width);
                    if(error <= 10 && error >= -10){
                        error = 0.0;
                    }
                    Imgproc.rectangle(submat, redRect, CONTOUR_COLOR_CONES, 2);
                    Imgproc.putText(submat, "Red Cone", new Point(redRect.x, redRect.y < 10 ? (redRect.y+redRect.height+20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR_CONES, 1);
                }
            }

            maskRed.release();
            erodeRed.release();
            dilateRed.release();
        }

        else {
            inRange(labCones, blueLow, blueHigh, maskBlue);
            Imgproc.erode(maskBlue, erodeBlue, kernel);
            Imgproc.dilate(erodeBlue, dilateBlue, kernel);

            blueContours.clear();

            Imgproc.findContours(dilateBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                blueContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon_cones
                        || (Imgproc.boundingRect(c).y > 400 && (Imgproc.boundingRect(c).height)/(Imgproc.boundingRect(c).width) < 1));
            }
            Imgproc.drawContours(submat, blueContours, -1, CONTOUR_COLOR_CONES);

            if(!blueContours.isEmpty()) {
                MatOfPoint biggestBlueContour = null;
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                    biggestBlueContour = Collections.max(blueContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                }
                if(Imgproc.contourArea(biggestBlueContour) > CONTOUR_AREA_CONES) {

                    blueRect = Imgproc.boundingRect(biggestBlueContour);
                    error = 450 - (blueRect.width / 2 + blueRect.x);

                    Log.i("error: ", "" + error);
                    Log.i("height/width: ", "" + blueRect.height/blueRect.width);
                    if(error <= 15 && error >= -15){
                        error = 0.0;
                    }
                    Imgproc.rectangle(submat, blueRect, CONTOUR_COLOR_CONES, 2);
                    Imgproc.putText(submat, "Red Cone", new Point(blueRect.x, blueRect.y < 10 ?
                            (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 2, TEXT_COLOR_CONES, 1);
                }
            }

            maskBlue.release();
            erodeBlue.release();
            dilateBlue.release();
        }

        Imgproc.line(submat, new Point(0, horizon_cones), new Point(640, horizon_cones), HORIZON_COLOR_CONES);

        labCones.release();

        if (savePictureFlag) {
            Log.i("TAG", "savePictureFlag");
            //FtcMatchInfo matchInfo = FtcMatchInfo.getMatchInfo();
            String matchName = String.format(Locale.US, "%s", "PICTURE01");
            saveMatToDisk(source0, matchName);
            savePictureFlag = false;
        }

        return source0;
    }

    public double getError(){
        return error;
    }

    public void detectRed(){
        detectRed = true;
    }

    public void detectBlue(){
        detectRed = false;
    }

    public void savePicture() {
        savePictureFlag = true;
        Log.i("TAG", "savePictureCall");
    }
}