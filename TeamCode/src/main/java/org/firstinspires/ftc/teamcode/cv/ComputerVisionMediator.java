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
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.XKd;
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.XKp;
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.YKd;
import static org.firstinspires.ftc.teamcode.drive.constants.PIDConstants.YKp;

import javax.annotation.Nullable;

public class ComputerVisionMediator {

    MecanumDrive drive;

    public VisionPortal visionPortal;
    private VisionPortal.Builder builder;

    private AprilTagProcessor aprilTag;
    private IMU imu;

    private LinearOpMode opMode;

    // Initialization for red
    public void init(HardwareMap hardwareMap, @Nullable MecanumDrive mecanumDrive, RedOctopusPipeline octopusPipeline, boolean useAprilTag, LinearOpMode linearOpMode) {
        opMode = linearOpMode;
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

        if (useAprilTag) {
            initAprilTagCV();
        } else {
            initCV(hardwareMap, octopusPipeline);
        }

        visionPortal = builder.build();
    }

    // Initialization for blue
    public void init(HardwareMap hardwareMap, @Nullable MecanumDrive mecanumDrive, BlueOctopusPipeline octopusPipeline, boolean useAprilTag, LinearOpMode linearOpMode) {
        opMode = linearOpMode;
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

        if (useAprilTag) {
            initAprilTagCV();
        } else {
            initCV(hardwareMap, octopusPipeline);
        }

        visionPortal = builder.build();
    }

    // Align heading with april tag
    public class TurnAlign implements Action {
        /** @noinspection FieldCanBeLocal*/
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
                double hError = detection.ftcPose.yaw;
                if (detection.id == 1 || detection.id == 4) {
                    turnPID(hError);
                }
            }

            if (timer.seconds() > 1.9) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
            return timer.seconds() < 2;
        }
    }

    public class DistanceAlign implements Action {
        private boolean initialized = false;

        List<AprilTagDetection> currentDetections;
        private boolean finished = false;

        ElapsedTime timer = new ElapsedTime();
        double xPower;
        double yPower;
        double xError = 5;
        double yError = 5;

        double x_ERROR_THRESH = 0.5;
        double y_ERROR_THRESH = 5;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                visionPortal.setProcessorEnabled(aprilTag, true);
                timer.reset();
                initialized = true;
            }
            currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 3 || detection.id == 6) {
                    while ((Math.abs(xError) > x_ERROR_THRESH || Math.abs(yError) > y_ERROR_THRESH && timer.seconds() < 2) && opMode.opModeIsActive()) {
                        // calculate the error, regardless of the target
                        currentDetections = aprilTag.getDetections();
                        xError = detection.ftcPose.x;
                        yError = detection.ftcPose.y;
                        xPower = (xError * XKp) + XKd;
                        yPower = (yError * YKp) + YKd;

                        // note: power positive means turn right,
                        drive.setDrivePowers(
                                new PoseVelocity2d(
                                        new Vector2d(yPower, xPower),
                                        0
                                )
                        );
                    }
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(0, 0),
                                    0
                            )
                    );
                    finished = true;
                }
            }

            return finished;
        }
    }

    public void turnPID(double degrees) {
        ElapsedTime timer = new ElapsedTime();
        double power;
        double error = 1;

        // the turn direction should always be consistent with the input parameter.
        double turnDirection = degrees > 0 ? -1:1;
        imu.resetYaw();
        timer.reset();
        double YAW_ERROR_THRESH = 0.1;
        while (Math.abs(error) > YAW_ERROR_THRESH && timer.seconds() < 3 && opMode.opModeIsActive()) {
            // calculate the error , regardless of the target or current turn angle
            error = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) - Math.abs(degrees);
            power = (error * IMUKp) + IMUKd;

            // note: power positive means turn right,
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            power * turnDirection
                    )
            );
        }
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }

    public double getYawAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // VERY COOL ACTIONS
    public Action turn90(boolean reverse) {
        return telemetryPacket -> {
            turnPID(reverse ? -90:90);
            return false;
        };
    }
    public Action distanceAlign() { return new DistanceAlign(); }

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


    private void initCV(HardwareMap hardwareMap, BlueOctopusPipeline octopusPipeline) {
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

    private void initCV(HardwareMap hardwareMap, RedOctopusPipeline octopusPipeline) {
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
