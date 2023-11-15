package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.RedOctopusPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Alliance Backdrop Auto", group = "Backdrop Side")
public class RedSideAutoBackdrop extends LinearOpMode {

    OpenCvCamera camera1;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(24, 0))
                .strafeToConstantHeading(new Vector2d(24, -16))
                .strafeToConstantHeading(new Vector2d(16, -16))
                .strafeToConstantHeading(new Vector2d(0, 36))
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(32, 0))
                .strafeToConstantHeading(new Vector2d(24, 0))
                .strafeToConstantHeading(new Vector2d(0, 36))
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(24, 0))
                .strafeToConstantHeading(new Vector2d(24, 16))
                .strafeToConstantHeading(new Vector2d(16, 16))
                .strafeToConstantHeading(new Vector2d(0, 36))
                .build();

        // Live preview thing
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Grab the webcam from the config files
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Create an OpenCV camera using webcam1
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);

        // Attach the pipeline
        RedOctopusPipeline octopusPipeline = new RedOctopusPipeline();
        camera1.setPipeline(octopusPipeline);

        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                // If resolution does not match, it will crash
                camera1.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called should the camera not open
                telemetry.addData("fail", "bad");
                telemetry.update();
            }
        });

        // Display Telemetry
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Detection", octopusPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();
        timer1.reset();
        if (isStopRequested()) return;

        switch (octopusPipeline.getLocation()) {
            case NONE:
            case MIDDLE:
                Actions.runBlocking(runToCenterProp);
                break;
            case LEFT:
                Actions.runBlocking(runToLeftProp);
                break;
            case RIGHT:
                Actions.runBlocking(runToRightProp);
                break;
        }
    }
}
