package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity closeBotBlueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-61, 12, Math.toRadians(0)))
                                .forward(28)
                                .strafeRight(12)
                                .back(4)
                                .strafeTo(new Vector2d(-28, 48))
                                .turn(Math.toRadians(90))
                                .forward(2)
                                .waitSeconds(2)
                                .back(2)
                                .strafeTo(new Vector2d(-60, 48))
                                .build()
                );
        RoadRunnerBotEntity closeBotRedRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, 12, Math.toRadians(180)))
                                .forward(28)
                                .strafeRight(12)
                                .back(4)
                                .strafeTo(new Vector2d(32, 48))
                                .turn(Math.toRadians(-90))
                                .forward(2)
                                .waitSeconds(2)
                                .back(2)
                                .strafeTo(new Vector2d(60, 48))
                                .build()
                );

        // LEFT
        RoadRunnerBotEntity closeBotRedLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, 12, Math.toRadians(180)))
                                .forward(28)
                                .strafeLeft(12)
                                .back(4)
                                .strafeTo(new Vector2d(28, 48))
                                .turn(Math.toRadians(-90))
                                .forward(2)
                                .waitSeconds(2)
                                .back(2)
                                .strafeTo(new Vector2d(60, 48))
                                .build()
                );

        RoadRunnerBotEntity farBotBlueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-61, -36, Math.toRadians(0)))
                                .waitSeconds(4)
                                .forward(28)
                                .strafeRight(12)
                                .back(4)
                                .strafeLeft(12)
                                .forward(24)
                                .turn(Math.toRadians(90))
                                .strafeTo(new Vector2d(-12, 48))
                                .strafeLeft(16)
                                .forward(2)
                                .waitSeconds(2)
                                .back(2)
                                .build()
                );

        RoadRunnerBotEntity farBotRedRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, -36, Math.toRadians(180)))
                                .waitSeconds(4)
                                .forward(28)
                                .strafeRight(12)
                                .back(4)
                                .strafeLeft(12)
                                .forward(24)
                                .turn(Math.toRadians(-90))
                                .strafeTo(new Vector2d(12, 48))
                                .strafeRight(31)
                                .forward(2)
                                .waitSeconds(2)
                                .back(2)
                                .build()
                );

        // LEFT
        RoadRunnerBotEntity farBotRedLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(61, -36, Math.toRadians(180)))
                                .waitSeconds(4)
                                .forward(28)
                                .strafeLeft(12)
                                .back(4)
                                .strafeRight(12)
                                .forward(24)
                                .turn(Math.toRadians(-90))
                                .strafeTo(new Vector2d(12, 48))
                                .strafeRight(16)
                                .forward(2)
                                .waitSeconds(2)
                                .back(2)
                                .build()
                );



        // Image must be in PNG format! (Webp is sucks)
        // Locates my field image on both my programming devices
        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\A\\Documents\\cs-field.png")); }
        catch (IOException e) {
            try { img = ImageIO.read(new File("C:\\Users\\pecke\\Documents\\cs-field.png")); }
            catch (IOException e2) {
                System.out.println("Your PNG image was not found");
            }
        }


        meepMeep.getWindowFrame().setResizable(true);

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(closeBotBlueRight)
                .addEntity(farBotBlueRight)
                .addEntity(closeBotRedRight)
                .addEntity(farBotRedRight)
                .start();
    }
}