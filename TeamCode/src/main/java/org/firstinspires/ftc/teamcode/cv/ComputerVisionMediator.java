package org.firstinspires.ftc.teamcode.cv;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.IMUKp;
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.IMUKd;

public class ComputerVisionMediator {

    MecanumDrive drive;

    public VisionPortal visionPortal;
    private VisionPortal.Builder builder;

    private AprilTagProcessor aprilTag;
    private IMU imu;

    // Initialization for red
    public void init(HardwareMap hardwareMap, MecanumDrive mecanumDrive, RedOctopusPipeline octopusPipeline) {
        drive = mecanumDrive;
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);

        initCV(hardwareMap, octopusPipeline);
        initAprilTagCV();

        visionPortal = builder.build();
    }

    // Initialization for blue
    public void init(HardwareMap hardwareMap, MecanumDrive mecanumDrive, BlueOctopusPipeline octopusPipeline) {
        drive = mecanumDrive;
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);

        initCV(hardwareMap, octopusPipeline);
        initAprilTagCV();

        visionPortal = builder.build();
    }

    public class LeftTurnAlign implements Action {
        /** @noinspection FieldCanBeLocal*/
        private final double YAW_ERROR_THRESHOLD = 0.5;
        private boolean initialized = false;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                visionPortal.setProcessorEnabled(aprilTag, true);
                imu.resetYaw();
                timer.reset();
                initialized = true;
            }

            for (AprilTagDetection detection : currentDetections) {
                if ((detection.id == 1 || detection.id == 4) && timer.seconds() < 3) {
                    double error = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) - detection.ftcPose.yaw;
                    if (Math.abs(error) > YAW_ERROR_THRESHOLD) {
                        turnPID(error);
                    }
                }
                if (timer.seconds() > 3) {
                    break;
                }
            }

            return timer.seconds() < 3;
        }
    }

    public Action leftTurnAlign() {
        return new LeftTurnAlign();
    }

    private void turnPID(double error) {
        double turnDirection = error > 0 ? -1:1;
        double turnPower = (error * IMUKp) + IMUKd;

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        turnPower * turnDirection
                )
        );
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag(LinearOpMode opMode) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        opMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        opMode.telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


    public void initCV(HardwareMap hardwareMap, BlueOctopusPipeline octopusPipeline) {
        // Create the vision portal by using a builder.
        builder = new VisionPortal.Builder();

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
        builder.addProcessor(octopusPipeline);

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public void initCV(HardwareMap hardwareMap, RedOctopusPipeline octopusPipeline) {
        // Create the vision portal by using a builder.
        builder = new VisionPortal.Builder();

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
        builder.addProcessor(octopusPipeline);

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTagCV() {

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

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
    }
}
