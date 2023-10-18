package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedOctopusPipeline extends OpenCvPipeline {

    // Camera resolution (check camera)
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;

    // Variables for team prop location
    enum SpikeLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        NONE
    }

    SpikeLocation location;

    // Define regions for left, center, and right thirds
    int leftRegionStart = 0;
    int leftRegionEnd = WIDTH / 3;
    int centerRegionStart = WIDTH / 3;
    int centerRegionEnd = (2 * WIDTH) / 3;
    int rightRegionStart = (2 * WIDTH) / 3;
    int rightRegionEnd = WIDTH;

    @Override
    public Mat processFrame(Mat input) {

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no team prop
        if (mat.empty()) {
            location = SpikeLocation.NONE;
            return input;
        }

        // We create a HSV range for blue to detect the team prop
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(0, 100, 100); // lower bound HSV for red
        Scalar highHSV = new Scalar(130, 255, 255); // higher bound HSV for red

        Mat blueMask = new Mat();
        Core.inRange(mat, lowHSV, highHSV, blueMask);

        // Calculate the sum of non-zero elements in each region
        double leftSum = Core.sumElems(blueMask.submat(new Rect(leftRegionStart, 0, leftRegionEnd - leftRegionStart, HEIGHT))).val[0];
        double centerSum = Core.sumElems(blueMask.submat(new Rect(centerRegionStart, 0, centerRegionEnd - centerRegionStart, HEIGHT))).val[0];
        double rightSum = Core.sumElems(blueMask.submat(new Rect(rightRegionStart, 0, rightRegionEnd - rightRegionStart, HEIGHT))).val[0];

        // Detect if a red object is in one of the three regions
        if (leftSum > 0) {
            Imgproc.rectangle(mat, new Point(leftRegionStart, 0), new Point(leftRegionEnd, HEIGHT), new Scalar(0, 255, 255), 2);
            location = SpikeLocation.LEFT;
        } else if (centerSum > 0) {
            Imgproc.rectangle(mat, new Point(centerRegionStart, 0), new Point(centerRegionEnd, HEIGHT), new Scalar(0, 255, 255), 2);
            location = SpikeLocation.MIDDLE;
        } else if (rightSum > 0) {
            Imgproc.rectangle(mat, new Point(rightRegionStart, 0), new Point(rightRegionEnd, HEIGHT), new Scalar(0, 255, 255), 2);
            location = SpikeLocation.RIGHT;
        } else {
            location = SpikeLocation.NONE;
        }

        return mat;
    }

    public SpikeLocation getLocation() { return this.location; }
}
