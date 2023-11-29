package org.firstinspires.ftc.teamcode.cv;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class RedOctopusPipeline implements VisionProcessor {

    // Camera resolution (check camera)
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;

    // Variables for team prop location
    public enum SpikeLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        NONE
    }

    SpikeLocation location;
    private double leftSum = 0;
    private double centerSum = 0;
    private double rightSum = 0;

    // Define regions for left, center, and right thirds
    int leftRegionStart = 0;
    int leftRegionEnd = WIDTH / 3;
    int centerRegionStart = WIDTH / 3;
    int centerRegionEnd = (2 * WIDTH) / 3;
    int rightRegionStart = (2 * WIDTH) / 3;
    int rightRegionEnd = WIDTH;

    // We create a HSV range for blue to detect the team prop
    // NOTE: In OpenCV's implementation,
    // Hue values are half the real value
    Scalar lowHSV = new Scalar(0, 40, 70); // lower bound HSV for red
    Scalar highHSV = new Scalar(20, 255, 255); // higher bound HSV for red

    // Paint for image preview
    Paint paint = new Paint();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        paint.setColor(Color.RED);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no team prop
        if (hsv.empty()) {
            location = SpikeLocation.NONE;
            return input;
        }

        Mat redMask = new Mat();
        Core.inRange(hsv, lowHSV, highHSV, redMask);

        // Filter out background noise using Gaussian blur
        Imgproc.GaussianBlur(redMask, redMask, new Size(9, 9), 2, 2);

        // Calculate the sum of non-zero elements in each region
        leftSum = Core.sumElems(redMask.submat(new Rect(leftRegionStart, 0, leftRegionEnd - leftRegionStart, HEIGHT))).val[0];
        centerSum = Core.sumElems(redMask.submat(new Rect(centerRegionStart, 0, centerRegionEnd - centerRegionStart, HEIGHT))).val[0];
        rightSum = Core.sumElems(redMask.submat(new Rect(rightRegionStart, 0, rightRegionEnd - rightRegionStart, HEIGHT))).val[0];

        // Release mats to conserve memory
        hsv.release();
        redMask.release();

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Detect if a red object is in one of the three regions
        if (leftSum > 0 && leftSum > centerSum && leftSum > rightSum) {
            canvas.drawRect(leftRegionStart, 0, leftRegionEnd, HEIGHT, paint);
            location = SpikeLocation.LEFT;
        } else if (centerSum > 0 && centerSum > leftSum && centerSum > rightSum) {
            canvas.drawRect(centerRegionStart, 0, centerRegionEnd, HEIGHT, paint);
            location = SpikeLocation.MIDDLE;
        } else if (rightSum > 0 && rightSum > leftSum && rightSum > centerSum) {
            canvas.drawRect(rightRegionStart, 0, rightRegionEnd, HEIGHT, paint);
            location = SpikeLocation.RIGHT;
        } else {
            location = SpikeLocation.NONE;
        }
    }

    public SpikeLocation getLocation() { return this.location; }
    public double getLeftSum() { return this.leftSum; }
    public double getCenterSum() { return this.centerSum; }
    public double getRightSum() { return this.rightSum; }


}
