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
import org.firstinspires.ftc.teamcode.cv.ComputerVisionMediator;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem2;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

@Autonomous(name = "Blue Alliance Substation Auto", group = "Substation Side")
public class BlueSideAutoSubstation extends LinearOpMode {

    BlueOctopusPipeline octopusPipeline = new BlueOctopusPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer1 = new ElapsedTime();

        // Initialize the drive
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ArmSubsystem2 arm = new ArmSubsystem2(hardwareMap);
        ComputerVisionMediator CVMediator = new ComputerVisionMediator();

        // Initialize some functions
        AutoFunctions functions = new AutoFunctions();
        functions.init(intake, arm, drive, false);

        // Change to torque motor for reliability
        arm.setCurrentSlides(arm.upperSlides);

        // Run to the left spike location
        Action runToLeftProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(28, 0))
                .strafeToConstantHeading(new Vector2d(28, -12))
                .strafeToConstantHeading(new Vector2d(24, -12))
                .build();
        // Run to the center spike location
        Action runToCenterProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(32, 0))
                .strafeToConstantHeading(new Vector2d(24, 0))
                .build();
        // Run to the right spike location
        Action runToRightProp = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(28, 0))
                .strafeToConstantHeading(new Vector2d(28, 12))
                .strafeToConstantHeading(new Vector2d(24, 12))
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

        switch (octopusPipeline.getLocation()) {
            case NONE:
            case MIDDLE:
                Actions.runBlocking(runToCenterProp);
                CVMediator.turnPID(90);
                break;
            case LEFT:
                Actions.runBlocking(runToLeftProp);
                CVMediator.turnPID(90);
                break;
            case RIGHT:
                Actions.runBlocking(runToRightProp);
                CVMediator.turnPID(90);
                break;
        }
        functions.intakePixel(drive.pose);
    }
}
