package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoCoordinates {
    public double ROTATED = Math.toRadians(90);
    public double STRAIGHT = Math.toRadians(0);

    public AutoCoordinates(boolean isRedSide) {
        if (isRedSide) {
            ROTATED = Math.toRadians(-90);
            
            backdropCenterPos = new Vector2d(26, 32);
            backdropLeftPos = new Vector2d(31, 32);
            backdropRightPos = new Vector2d(16.5, 32);

            parkInsidePos = new Vector2d(52, 28); // Rotated
            parkOutsidePos = new Vector2d(52, 28); // Rotated

            toBackdropFromPixelStack = new Vector2d(47, 76); // Rotated
        }
    }
    // Starting position
    public Pose2d startPos = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

    // Team prop exact coordinates
    public Vector2d propCenterPos = new Vector2d(32, 0); // Straight
    public Vector2d propLeftPos = new Vector2d(28, -12); // Straight
    public Vector2d propRightPos = new Vector2d(28, 12); // Straight

    // Around the team prop for side randomization
    public Vector2d betweenSidePropPos = new Vector2d(28, 0); // Run between the spike marks to strafe Straight
    public Vector2d backFromCenterPropPos = new Vector2d(24, 0); // Scoot back 8 inches from center prop Straight
    public Vector2d backFromLeftPropPos = new Vector2d(24, -12); // Scoot back 4 inches from left prop Straight
    public Vector2d backFromRightPropPos = new Vector2d(24, 12); // Scoot back 4 inches from right prop Straight

    // Backdrop coordinates for BLUE side (Do I rotate before or after?? Adjust the angle.)
    public Vector2d backdropCenterPos = new Vector2d(26, -32); // Straight
    public Vector2d backdropLeftPos = new Vector2d(16.5, -32); // Straight
    public Vector2d backdropRightPos = new Vector2d(31, -32); // Straight

    // Parking options for BLUE side
    public Vector2d parkInsidePos = new Vector2d(52, -28); // Rotated
    public Vector2d parkOutsidePos = new Vector2d(52, -28); // Rotated

    // Substation side stuff
    // Exists in AutoFunctions.java
    public Vector2d toBackdropFromPixelStack = new Vector2d(47, -76); // Rotated

}
