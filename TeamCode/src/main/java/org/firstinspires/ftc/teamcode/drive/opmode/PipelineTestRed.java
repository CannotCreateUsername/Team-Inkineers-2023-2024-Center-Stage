package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.firstinspires.ftc.teamcode.cv.RedOctopusPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Pipeline Test Red", group = "Concept")
public class PipelineTestRed extends LinearOpMode {

    OpenCvCamera camera1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Live preview thing
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Grab the webcam from the config files
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Create an OpenCV camera using webcam1
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);

        // Attach the pipeline
        RedOctopusPipeline octopusPipeline = new RedOctopusPipeline();
        camera1.setPipeline(octopusPipeline);

        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                // If resolution does not match, it will crash
                camera1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                // This will be called should the camera not open
                telemetry.addData("faile", "bad");
                telemetry.update();
            }
        });


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Zone", octopusPipeline.getLocation());
            telemetry.addData("Pixels in left", octopusPipeline.getLeftSum());
            telemetry.addData("Pixels in middle", octopusPipeline.getCenterSum());
            telemetry.addData("Pixels in right", octopusPipeline.getRightSum());
            telemetry.update();
        }
    }
}