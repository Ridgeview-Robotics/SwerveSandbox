package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

import org.openftc.easyopencv.OpenCvCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ALGrithym extends OpenCvPipeline {



    // Your target color range in HSV
    private final Scalar lowerBound = new Scalar(20, 100, 100); // adjust for your element
    private final Scalar upperBound = new Scalar(30, 255, 255);
    OpenCvCamera camera;
    GameElementDetector detector;
    // Store results so your OpMode can read them
    public double lastAspectRatio = 0;
    public double lastSolidity = 0;
    public boolean elementDetected = false;

    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        // Step 1: Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Step 2: Color mask
        Core.inRange(hsv, lowerBound, upperBound, mask);

        // Step 3: Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Step 4: Analyze each contour
        elementDetected = false;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 500) continue;

            // Aspect ratio
            Rect bbox = Imgproc.boundingRect(contour);
            double aspectRatio = (double) bbox.width / bbox.height;

            // Solidity — convexHull returns indices, so we need to convert to points
            MatOfInt hullIndices = new MatOfInt();
            Imgproc.convexHull(contour, hullIndices);

            Point[] contourPoints = contour.toArray();
            int[] hullIdx = hullIndices.toArray();
            MatOfPoint hullPoints = new MatOfPoint();
            Point[] hullPts = new Point[hullIdx.length];
            for (int i = 0; i < hullIdx.length; i++) {
                hullPts[i] = contourPoints[hullIdx[i]];
            }
            hullPoints.fromArray(hullPts);

            double hullArea = Imgproc.contourArea(hullPoints);
            double solidity = area / hullArea;

            // Filter based on shape
            if (aspectRatio < 0.5 || aspectRatio > 2.0) continue; // not squarish
            if (solidity < 0.8) continue;                          // too irregular

            // If it passed both checks, it's likely a real game element
            Imgproc.rectangle(input, bbox, new Scalar(0, 255, 0), 2);
            elementDetected = true;
            lastAspectRatio = aspectRatio;
            lastSolidity = solidity;
        }

        return input; // returns the annotated frame to the Driver Station
    }
}