package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cv.AprilTagMediator;

@TeleOp(name = "Dual CV Testing", group = "CV")
public class DualCVTester extends LinearOpMode {

    MecanumDrive drive;
    AprilTagMediator aprilTagMediator = new AprilTagMediator();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        aprilTagMediator.init(hardwareMap, drive);


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
}
