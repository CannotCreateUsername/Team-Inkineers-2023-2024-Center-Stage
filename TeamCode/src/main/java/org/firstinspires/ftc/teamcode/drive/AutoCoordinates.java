package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoCoordinates {

    // Team prop exact coordinates
    public Pose2d centerPropPos = new Pose2d(new Vector2d(32, 0), Math.toRadians(0));
    public Pose2d leftPropPos = new Pose2d(new Vector2d(28, -12), Math.toRadians(0));
    public Pose2d rightPropPos = new Pose2d(new Vector2d(28, 12), Math.toRadians(0));

    // Around the team prop for side randomization
    public Pose2d sidePropReadyPos = new Pose2d(new Vector2d(28, 0), Math.toRadians(0)); // Run between the spike marks to strafe
    public Pose2d backFromLeftPropPos = new Pose2d(new Vector2d(24, -12), Math.toRadians(0)); // Scoot back 4 inches from left prop
    public Pose2d backFromRightPropPos = new Pose2d(new Vector2d(24, 12), Math.toRadians(0)); // Scoot back 4 inches from right prop

    // Backdrop coordinates for BLUE side (Do I rotate before or after?? Adjust the angle.)
    public Pose2d blueCenterBackdropPos = new Pose2d(new Vector2d(26, -32), Math.toRadians(0)); // Center position for backdrop
    public Pose2d blueLeftBackdropPos = new Pose2d(new Vector2d(16.5, -32), Math.toRadians(0)); // Left position for backdrop
    public Pose2d blueRightBackdropPos = new Pose2d(new Vector2d(31, -32), Math.toRadians(0)); // Right position for backdrop

    // Backdrop coordinates for RED side (Do I rotate before or after?? Adjust the angle.)
    public Pose2d redCenterBackdropPos = new Pose2d(new Vector2d(26, 32), Math.toRadians(0)); // Center position for backdrop
    public Pose2d redLeftBackdropPos = new Pose2d(new Vector2d(31, 32), Math.toRadians(0)); // Left position for backdrop
    public Pose2d redRightBackdropPos = new Pose2d(new Vector2d(16.5, 32), Math.toRadians(0)); // Right position for backdrop

    // Parking options for BLUE side
    public Pose2d blueInsideParkPos = new Pose2d(new Vector2d(52, -28), Math.toRadians(90)); // Inside parking position
    public Pose2d blueOutsideParkPos = new Pose2d(new Vector2d(52, -28), Math.toRadians(90)); // Outside parking position

    // Parking options for RED side
    public Pose2d redInsideParkPos = new Pose2d(new Vector2d(52, 28), Math.toRadians(-90)); // Inside parking position
    public Pose2d redOutsideParkPos = new Pose2d(new Vector2d(52, 28), Math.toRadians(-90)); // Outside parking position

    // Substation side stuff
    // Exists in AutoFunctions.java
}
