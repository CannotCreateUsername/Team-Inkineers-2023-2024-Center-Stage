package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public class AutoCoordinates {
    public double ROTATED = Math.toRadians(90);
    public double STRAIGHT = Math.toRadians(0);

    public AutoCoordinates(boolean isRedSide) {
        if (isRedSide) {
            ROTATED = Math.toRadians(-90);
            
            backdropCenterPos = new Vector2d(18, -42);
            backdropLeftPos = new Vector2d(23, -42);
            backdropRightPos = new Vector2d(13, -42);

            afterDropCenter = new Vector2d(18, -44);
            afterDropLeft = new Vector2d(23, -44);
            afterDropRight = new Vector2d(13, -44);

            parkInsidePos = new Vector2d(52, -38); // Rotated
            parkOutsidePos = new Vector2d(-6, -38); // Rotated

            // Substation Stuff
            beforePixelCrash = new Vector2d(16, 12); // Rotated
            pixelStackPos = new Vector2d(45, 16); // Rotated
            backIntoPixelPos = new Vector2d(45, 18); // Rotated
            backToIntakePixel = new Vector2d(45, 20); // Rotated

            toBackdropFromPixelStack = new Vector2d(45, -84); // Rotated
            betweenSubBackdrop = new Vector2d(15, -84); // Rotated
            subLeftBackdrop = new Vector2d(20, -97); // Rotated
            subCenterBackdrop = new Vector2d(15, -97); // Rotated
            subRightBackdrop = new Vector2d(6, -97); // Rotated

            subParkPos = new Vector2d(52,-88); // Rotated
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
    public Vector2d backdropCenterPos = new Vector2d(18, 42); // Rotated
    public Vector2d backdropLeftPos = new Vector2d(13, 42); // Rotated
    public Vector2d backdropRightPos = new Vector2d(23, 42); // Rotated

    public Vector2d afterDropCenter = new Vector2d(18, 44); // Rotated
    public Vector2d afterDropLeft = new Vector2d(13, 44); // Rotated
    public Vector2d afterDropRight = new Vector2d(23, 44); // Rotated

    // Parking options for BLUE side
    public Vector2d parkInsidePos = new Vector2d(52, 38); // Rotated
    public Vector2d parkOutsidePos = new Vector2d(-6, 38); // Rotated


    // Substation side stuff
    public Vector2d beforePixelCrash = new Vector2d(16, -12); // Rotated
    public Vector2d pixelStackPos = new Vector2d(45, -16); // Rotated
    public Vector2d backIntoPixelPos = new Vector2d(45, -18); // Rotated
    public Vector2d backToIntakePixel = new Vector2d(45, -20); // Rotated

    public Vector2d toBackdropFromPixelStack = new Vector2d(45, 84); // Rotated
    public Vector2d betweenSubBackdrop = new Vector2d(15, 84); // Rotated

    public Vector2d subLeftBackdrop = new Vector2d(6, 97); // Rotated
    public Vector2d subCenterBackdrop = new Vector2d(15, 97); // Rotated
    public Vector2d subRightBackdrop = new Vector2d(20, 97); // Rotated

    public Vector2d subParkPos = new Vector2d(52,88); // Rotated
}
