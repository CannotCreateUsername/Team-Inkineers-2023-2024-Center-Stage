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

        // Create the vision portal by using a builder.
        builder = new VisionPortal.Builder();
        // Set the webcam that the vision portal will use.
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        if (useAprilTag) {
            initAprilTagCV();
        } else {
            initCV(octopusPipeline);
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

        // Create the vision portal by using a builder.
        builder = new VisionPortal.Builder();
        // Set the webcam that the vision portal will use.
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        if (useAprilTag) {
            initAprilTagCV();
        } else {
            initCV(octopusPipeline);
        }

        visionPortal = builder.build();
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
        while (timer.seconds() < 3 && opMode.opModeIsActive()) {
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

    private void initCV(BlueOctopusPipeline blueOctopusPipeline) {
        // Set and enable the processor.
        builder.addProcessor(blueOctopusPipeline);
    }

    private void initCV(RedOctopusPipeline redOctopusPipeline) {
        // Set and enable the processor.
        builder.addProcessor(redOctopusPipeline);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTagCV() {

        // Create the AprilTag processor.
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
                .setLensIntrinsics(603.684, 603.684, 343.776, 243.074) // 4/1/24
                // 603.245, 603.245, 301.575, 233.437 - Pre 4/1/24
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
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
}
