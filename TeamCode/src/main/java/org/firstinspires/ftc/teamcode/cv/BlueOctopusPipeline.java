package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueOctopusPipeline extends OpenCvPipeline {

    // Camera resolution (check camera)
    public static final int WIDTH = 1;

    // Variables for team prop location
    enum SpikeLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        NONE
    }

    SpikeLocation location;

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
        Scalar lowHSV = new Scalar(105, 255*.75, 255*.3); // lower bound HSV for blue
        Scalar highHSV = new Scalar(130, 255, 255); // higher bound HSV for blue
        Mat thresh = new Mat();

        Core.inRange(mat, lowHSV, highHSV, thresh);

        return mat;
    }

    public SpikeLocation getLocation() { return this.location; }
}
