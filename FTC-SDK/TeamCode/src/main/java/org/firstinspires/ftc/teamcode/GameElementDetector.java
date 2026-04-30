package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class GameElementDetector extends OpenCvPipeline {

    // Color ranges in HSV — hue range + min saturation of 50, brightness ignored
    private final Scalar yellowLow  = new Scalar(20,  50, 0);
    private final Scalar yellowHigh = new Scalar(30,  255, 255);

    private final Scalar blueLow    = new Scalar(90,  50, 0);
    private final Scalar blueHigh   = new Scalar(140, 255, 255);

    private final Scalar greenLow   = new Scalar(40,  50, 0);
    private final Scalar greenHigh  = new Scalar(80,  255, 255);

    private final Scalar purpleLow  = new Scalar(130, 50, 0);
    private final Scalar purpleHigh = new Scalar(160, 255, 255);

    private final Scalar redLow1    = new Scalar(0,   50, 0);
    private final Scalar redHigh1   = new Scalar(15,  255, 255);
    private final Scalar redLow2    = new Scalar(160, 50, 0);
    private final Scalar redHigh2   = new Scalar(180, 255, 255);

    public double lastAspectRatio = 0;
    public double lastSolidity = 0;
    public boolean elementDetected = false;
    public double lastHue = 0;
    public String lastColor = "none";

    private Mat hsv       = new Mat();
    private Mat mask      = new Mat();
    private Mat hierarchy = new Mat();
    private Mat redMask1  = new Mat();
    private Mat redMask2  = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat yellowMask = new Mat();
        Mat blueMask   = new Mat();
        Mat greenMask  = new Mat();
        Mat purpleMask = new Mat();

        Core.inRange(hsv, yellowLow,  yellowHigh,  yellowMask);
        Core.inRange(hsv, blueLow,    blueHigh,    blueMask);
        Core.inRange(hsv, greenLow,   greenHigh,   greenMask);
        Core.inRange(hsv, purpleLow,  purpleHigh,  purpleMask);
        Core.inRange(hsv, redLow1,    redHigh1,    redMask1);
        Core.inRange(hsv, redLow2,    redHigh2,    redMask2);

        Core.add(redMask1, redMask2, mask);
        Core.add(mask, yellowMask, mask);
        Core.add(mask, blueMask,   mask);
        Core.add(mask, greenMask,  mask);
        Core.add(mask, purpleMask, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        elementDetected = false;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 500) continue;

            Rect bbox = Imgproc.boundingRect(contour);
            double aspectRatio = (double) bbox.width / bbox.height;

            Mat region = new Mat(hsv, bbox);
            Scalar mean = Core.mean(region);
            lastHue = mean.val[0];

            MatOfInt hullIndices = new MatOfInt();
            Imgproc.convexHull(contour, hullIndices);

            Point[] contourPoints = contour.toArray();
            int[] hullIdx = hullIndices.toArray();
            Point[] hullPts = new Point[hullIdx.length];
            for (int i = 0; i < hullIdx.length; i++) {
                hullPts[i] = contourPoints[hullIdx[i]];
            }
            MatOfPoint hullPoints = new MatOfPoint();
            hullPoints.fromArray(hullPts);

            double hullArea = Imgproc.contourArea(hullPoints);
            double solidity = area / hullArea;

            if (aspectRatio < 0.5 || aspectRatio > 2.0) continue;
            if (solidity < 0.8) continue;

            // Identify color from hue value
            Scalar boxColor;
            if (lastHue < 28 || lastHue > 170) {
                lastColor = "red";
                boxColor = new Scalar(255, 0, 0);
            } else if (lastHue >= 28 && lastHue <= 40) {
                lastColor = "yellow";
                boxColor = new Scalar(255, 255, 0);
            } else if (lastHue >= 40 && lastHue <= 80) {
                lastColor = "green";
                boxColor = new Scalar(0, 255, 0);
            } else if (lastHue >= 90 && lastHue <= 105.) {
                lastColor = "blue";
                boxColor = new Scalar(0, 0, 255);
            } else if (lastHue >= 106 && lastHue <= 160) {
                lastColor = "purple";
                boxColor = new Scalar(128, 0, 128);
            } else {
                lastColor = "unknown";
                boxColor = new Scalar(255, 255, 255);
            }

            Imgproc.rectangle(input, bbox, boxColor, 2);
            elementDetected = true;
            lastAspectRatio = aspectRatio;
            lastSolidity = solidity;
        }

        return input;
    }
}