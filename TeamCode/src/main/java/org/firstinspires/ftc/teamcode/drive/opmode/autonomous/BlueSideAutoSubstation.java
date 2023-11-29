package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;

@Autonomous(name = "Blue Alliance Substation Auto", group = "Substation Side")
public class BlueSideAutoSubstation extends LinearOpMode {

    BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(28, 0))
                .strafeToConstantHeading(new Vector2d(28, -12))
                .strafeToConstantHeading(new Vector2d(24, -12))
                .turn(Math.toRadians(90))
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(31, 0))
                .strafeToConstantHeading(new Vector2d(24, 0))
                .turn(Math.toRadians(90))
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(28, 0))
                .strafeToConstantHeading(new Vector2d(28, 12))
                .strafeToConstantHeading(new Vector2d(24, 12))
                .turn(Math.toRadians(90))
                .build();

        // Display Telemetry
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Detection", octopusPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        timer1.reset();
        if (isStopRequested()) return;

        // Stop the pipeline since we no longer need to detect the prop
//        CVMediator.visionPortal.setProcessorEnabled(octopusPipeline, false);


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
