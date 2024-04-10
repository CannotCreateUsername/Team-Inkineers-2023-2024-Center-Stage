package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.backdrop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.BlueOctopusPipeline;
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.AutoCoordinates;
import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.AutoFunctions;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem5;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@Autonomous(name = "BLUE Backdrop Park/WALL", group = "Backdrop Side")
public class BlueSideAutoBackdrop extends LinearOpMode {

    BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        // Get the coordinates
        AutoCoordinates coords = new AutoCoordinates(false, false);

        // Initialize the drive
        Pose2d startPose = coords.startPos;
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmSubsystem5 arm = new ArmSubsystem5(hardwareMap);
        ComputerVisionMediator CVMediator = new ComputerVisionMediator();

        // Initialize some functions
        AutoFunctions functions = new AutoFunctions();
        functions.init(new IntakeSubsystem(hardwareMap), arm, drive);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propLeftPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromLeftProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backdropLeftPos, coords.ROTATED)
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.propCenterPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromCenterProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backdropCenterPos, coords.ROTATED)
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propRightPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromRightProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backdropRightPos, coords.ROTATED)
                .build();

        // Initialize all computer vision stuff
        CVMediator.init(hardwareMap, drive, octopusPipeline, false, this);

        // Display Telemetry
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Detection", octopusPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();
        timer1.reset();
        if (isStopRequested()) return;
        // OPMODE STARTS HERE
        // Stop the pipeline since we no longer need to detect the prop
        CVMediator.visionPortal.setProcessorEnabled(octopusPipeline, false);
        CVMediator.disableAprilTag();
        Vector2d beginParkPos = coords.afterDropCenter;

        switch (octopusPipeline.getLocation()) {
            case NONE:
            case MIDDLE:
                Actions.runBlocking(runToCenterProp);
                break;
            case LEFT:
                beginParkPos = coords.afterDropLeft;
                Actions.runBlocking(runToLeftProp);
                break;
            case RIGHT:
                beginParkPos = coords.afterDropRight;
                Actions.runBlocking(runToRightProp);
                break;
        }
        // Build parking path for each position from backdrop
        Action runToPark = drive.actionBuilder(new Pose2d(beginParkPos, coords.ROTATED))
                .strafeToConstantHeading(coords.parkOutsidePos)
                .build();

        // Finish scoring
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        functions.touchBackdrop(),
                        arm.dropYellowPixel(false)
                ),
                new ParallelAction(
                        arm.reset4Bar(),
                        arm.resetSlides(),
                        runToPark
                )
        ));
    }
}
