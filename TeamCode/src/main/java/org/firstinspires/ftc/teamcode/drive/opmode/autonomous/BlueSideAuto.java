package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Alliance Auto", group = "Linear Opmode")
public class BlueSideAuto extends LinearOpMode {

    OpenCvCamera camera1;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Live preview thing
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Grab the webcam from the config files
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Create an OpenCV camera using webcam1
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);

        // Attach the pipeline
        BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();
        camera1.setPipeline(octopusPipeline);

        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                // If resolution does not match, it will crash
                camera1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called should the camera not open
                telemetry.addData("fail", "bad");
                telemetry.update();
            }
        });

        waitForStart();
        switch (octopusPipeline.getLocation()) {
            case NONE:

        }
    }
}
