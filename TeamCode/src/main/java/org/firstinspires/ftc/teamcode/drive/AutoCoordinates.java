package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoCoordinates {
    public double ROTATED = Math.toRadians(90);
    public double STRAIGHT = Math.toRadians(0);

    public AutoCoordinates(boolean isRedSide) {
        if (isRedSide) {
            ROTATED = Math.toRadians(-90);
            
            backdropCenterPos = new Vector2d(28, -32);
            backdropLeftPos = new Vector2d(33, -32);
            backdropRightPos = new Vector2d(20, -32);

            afterDropCenter = new Vector2d(28, -30);
            afterDropLeft = new Vector2d(33, -34);
            afterDropRight = new Vector2d(20, -34);

            parkInsidePos = new Vector2d(52, -30); // Rotated
            parkOutsidePos = new Vector2d(8, -30); // Rotated

            toBackdropFromPixelStack = new Vector2d(47, -76); // Rotated
        }
    }
    // Starting position
    public Pose2d startPos = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

    // Team prop exact coordinates
    public Vector2d propCenterPos = new Vector2d(32, 0); // Straight
    public Vector2d propLeftPos = new Vector2d(29, 12); // Straight
    public Vector2d propRightPos = new Vector2d(29, -12); // Straight

    // Around the team prop for side randomization
    public Vector2d betweenSideProp = new Vector2d(29, 0); // Run between the spike marks to strafe Straight
    public Vector2d backFromCenterProp = new Vector2d(25, 0); // Scoot back 8 inches from center prop Straight
    public Vector2d backFromLeftProp = new Vector2d(25, 12); // Scoot back 4 inches from left prop Straight
    public Vector2d backFromRightProp = new Vector2d(25, -12); // Scoot back 4 inches from right prop Straight

    // Backdrop coordinates for BLUE side (Do I rotate before or after?? Adjust the orientation.)
    public Vector2d backdropCenterPos = new Vector2d(28, 30); // Straight
    public Vector2d backdropLeftPos = new Vector2d(20, 30); // Straight
    public Vector2d backdropRightPos = new Vector2d(33, 30); // Straight

    public Vector2d afterDropCenter = new Vector2d(28, 34); // Rotated
    public Vector2d afterDropLeft = new Vector2d(20, 34); // Rotated
    public Vector2d afterDropRight = new Vector2d(33, 34); // Rotated

    // Parking options for BLUE side
    public Vector2d parkInsidePos = new Vector2d(52, 30); // Rotated
    public Vector2d parkOutsidePos = new Vector2d(8, 30); // Rotated

    // Substation side stuff
    // Exists in AutoFunctions.java
    public Vector2d beforeCrossTrussPos = new Vector2d(48, 0);
    public Vector2d afterCrossTrussPos = new Vector2d(48, 76);

    public Vector2d toBackdropFromPixelStack = new Vector2d(47, 76); // Rotated

}
