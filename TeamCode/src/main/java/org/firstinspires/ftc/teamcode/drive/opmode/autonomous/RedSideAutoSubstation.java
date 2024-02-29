package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.cv.RedOctopusPipeline;
import org.firstinspires.ftc.teamcode.drive.AutoCoordinates;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem3;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@Autonomous(name = "RED Substation Park/Middle", group = "Substation Side")
public class RedSideAutoSubstation extends LinearOpMode {

    RedOctopusPipeline octopusPipeline = new RedOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        AutoCoordinates coords = new AutoCoordinates(true);

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmSubsystem3 arm = new ArmSubsystem3(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ComputerVisionMediator CVMediator = new ComputerVisionMediator();

        // Initialize some functions
        AutoFunctions functions = new AutoFunctions();
        functions.init(intake, arm, drive, false);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propLeftPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.beforePixelCrash, coords.ROTATED)
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.propCenterPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.backFromCenterProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.beforePixelCrash, coords.ROTATED)
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(startPose)
                .strafeToLinearHeading(coords.betweenSideProp, coords.STRAIGHT)
                .strafeToLinearHeading(coords.propRightPos, coords.STRAIGHT)
                .strafeToLinearHeading(coords.beforePixelCrash, coords.ROTATED)
                .build();
        // Run to the pixel (spin intake at same time)
        Action runToPixelStack = drive.actionBuilder(new Pose2d(coords.beforePixelCrash, coords.ROTATED))
                .strafeToLinearHeading(coords.pixelStackPos, coords.ROTATED)
                .strafeToLinearHeading(coords.backIntoPixelPos, coords.ROTATED)
                .waitSeconds(0.5)
                .strafeToLinearHeading(coords.backToIntakePixel, coords.ROTATED)
                .waitSeconds(1)
                .strafeToLinearHeading(coords.pixelStackPos, coords.ROTATED)
                .strafeToLinearHeading(coords.backToIntakePixel, coords.ROTATED)
                .strafeToLinearHeading(coords.pixelStackPos, coords.ROTATED)
                .build();

        Action runAcrossField = drive.actionBuilder(new Pose2d(coords.pixelStackPos, coords.ROTATED))
                .strafeToLinearHeading(coords.toBackdropFromPixelStack, coords.ROTATED)
                .build();
        // Run to scoring on backdrop
        Action runToScoreCenter1 = drive.actionBuilder(new Pose2d(coords.toBackdropFromPixelStack, coords.ROTATED))
                .strafeToLinearHeading(coords.subCenterBackdrop, coords.ROTATED)
                .waitSeconds(3)
                .strafeToLinearHeading(coords.betweenSubBackdrop, coords.ROTATED)
                .build();
        Action runToScoreLeft1 = drive.actionBuilder(new Pose2d(coords.toBackdropFromPixelStack, coords.ROTATED))
                .strafeToLinearHeading(coords.subLeftBackdrop, coords.ROTATED)
                .waitSeconds(3)
                .strafeToLinearHeading(coords.betweenSubBackdrop, coords.ROTATED)
                .build();
        Action runToScoreRight1 = drive.actionBuilder(new Pose2d(coords.toBackdropFromPixelStack, coords.ROTATED))
                .strafeToLinearHeading(coords.subRightBackdrop, coords.ROTATED)
                .waitSeconds(3)
                .strafeToLinearHeading(coords.betweenSubBackdrop, coords.ROTATED)
                .build();
        // Second pixel (Yellow)
        Action runToScoreCenter2 = drive.actionBuilder(new Pose2d(coords.betweenSubBackdrop, coords.ROTATED))
                .strafeToLinearHeading(coords.subCenterBackdrop, coords.ROTATED)
                .build();
        Action runToScoreLeft2 = drive.actionBuilder(new Pose2d(coords.betweenSubBackdrop, coords.ROTATED))
                .strafeToLinearHeading(coords.subLeftBackdrop, coords.ROTATED)
                .build();
        Action runToScoreRight2 = drive.actionBuilder(new Pose2d(coords.betweenSubBackdrop, coords.ROTATED))
                .strafeToLinearHeading(coords.subRightBackdrop, coords.ROTATED)
                .build();

        CVMediator.init(hardwareMap, drive, octopusPipeline, false, this);

        // Display Telemetry
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Detection", octopusPipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        timer1.reset();
        if (isStopRequested()) return;

        // Store which path to take
        Action runToScoreYellow = runToScoreCenter2;
        Action runToScoreWhite = runToScoreLeft1;

        switch (octopusPipeline.getLocation()) {
            case NONE:
            case MIDDLE:
                Actions.runBlocking(new SequentialAction(
                        runToCenterProp,
                        new ParallelAction(
                                runToPixelStack,
                                functions.intakePixel()
                        )
                ));
                break;
            case LEFT:
                runToScoreYellow = runToScoreLeft2;
                runToScoreWhite = runToScoreCenter1;
                Actions.runBlocking(new SequentialAction(
                        runToLeftProp,
                        new ParallelAction(
                                runToPixelStack,
                                functions.intakePixel()
                        )
                ));
                break;
            case RIGHT:
                runToScoreYellow = runToScoreRight2;
                runToScoreWhite = runToScoreCenter1;
                Actions.runBlocking(new SequentialAction(
                        runToRightProp,
                        new ParallelAction(
                                runToPixelStack,
                                functions.intakePixel()
                        )
                ));
                break;
        }
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        runAcrossField,
                        intake.spinIntake(-0.8, 3)
                ),
                new ParallelAction(
                        runToScoreWhite,
                        new SequentialAction(
                                arm.readySlides(false),
                                arm.ready4bar(),
                                arm.spinOuttake(-0.5, 0.4)
                        )
                ),
                arm.readySlides(true),
                new ParallelAction(
                        runToScoreYellow
                ),
                arm.spinOuttake(-1, 0.5),
                arm.reset4Bar(),
                arm.resetSlides()
        ));
    }
}
