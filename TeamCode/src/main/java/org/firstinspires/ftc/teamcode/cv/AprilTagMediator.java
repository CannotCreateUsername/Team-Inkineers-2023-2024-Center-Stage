package org.firstinspires.ftc.teamcode.cv;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.ImuKp;
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.ImuKd;

public class AprilTagMediator {

    private double YAW_ERROR_THRESHOLD = 0.5;


    MecanumDrive drive;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final IMU imu;

    public AprilTagMediator(HardwareMap hardwareMap, MecanumDrive mecanumDrive) {
        drive = mecanumDrive;
        imu = hardwareMap.get(IMU.class, "IMU");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);

        initAprilTag(hardwareMap);
        visionPortal.stopStreaming();
    }

    public class LeftTurnAlign implements Action {
        private boolean initialized = false;
        private boolean finished = false;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                visionPortal.resumeStreaming();
                imu.resetYaw();
                timer.reset();
                initialized = true;
            }

            for (AprilTagDetection detection : currentDetections) {
                if ((detection.id == 1 || detection.id == 4) && timer.seconds() < 3) {

                }
            }

            return timer.seconds() < 3;
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        // ... these parameters are fx, fy, cx, cy.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(603.245, 603.245, 301.575, 233.437)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        // builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }
}
