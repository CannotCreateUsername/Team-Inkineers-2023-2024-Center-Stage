package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "CV Test", group = "Concept")
public class PipelineTest extends LinearOpMode {

    OpenCvCamera camera1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Live preview thing
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Grab the webcam from the config files
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "webcam1");
        // Create an OpenCV camera using webcam1
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);

        // Attach the pipeline
        BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();
        camera1.setPipeline(octopusPipeline);

        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                // This will be called should the camera not open
            }
        });


        waitForStart();
        while (opModeIsActive()) {

            telemetry.update();
        }
    }
}