package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.AprilTagMediator;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Dual CV Testing", group = "CV")
public class DualCVTester extends LinearOpMode {

    OpenCvCamera camera1;
    BlueOctopusPipeline octopusPipeline;
    MecanumDrive drive;
    AprilTagMediator aprilTagMediator = new AprilTagMediator();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));

        initCV();
        ElapsedTime cringeTimer = new ElapsedTime();

        waitForStart();
        cringeTimer.reset();
        while (opModeIsActive()) {
            telemetry.addLine("Never gonna");

            if (cringeTimer.seconds() > 1) {
                telemetry.addLine("give you up");
            }
            if (cringeTimer.seconds() > 1.5) {
                telemetry.addLine("Never gonna");
            }
            if (cringeTimer.seconds() > 2) {
                telemetry.addLine("let you down");
            }
            if (cringeTimer.seconds() > 2.5) {
                telemetry.addLine("Never gonna");
            }
            if (cringeTimer.seconds() > 3) {
                telemetry.addLine("turn around");
            }
            if (cringeTimer.seconds() > 3.5) {
                telemetry.addLine("And hurt you");
            }
            if (cringeTimer.seconds() > 5) {
                telemetry.addLine("AJSHFDNjfasjkbnsdagjkrsrngvrnKLNFKLSVKLKLNVklfdngjkrgsEKFiy489t5u89iojklJMEKDFMK:m3mtFLEMSDJLENFksejmkVMSD:LPLF{#PREPF{KFOI&T*$&%Yuiher%KEshdrfbsodiSHT$BIJKsernhfjksh5tjkz5N$JKRKNGNrKTN4wt");
            }
            telemetry.update();
        }
    }

    public void initCV() {
        // Live preview thing
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Grab the webcam from the config files
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Create an OpenCV camera using webcam1
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraMonitorViewId);

        // Attach the pipeline
        octopusPipeline = new BlueOctopusPipeline();
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

        aprilTagMediator.init(hardwareMap, drive);
    }
}
