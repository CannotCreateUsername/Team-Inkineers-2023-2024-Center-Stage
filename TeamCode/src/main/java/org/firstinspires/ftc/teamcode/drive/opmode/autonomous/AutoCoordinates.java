package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public class AutoCoordinates {
    public double ROTATED = Math.toRadians(90);
    public double STRAIGHT = Math.toRadians(0);

    public AutoCoordinates(boolean isRedSide) {
        if (isRedSide) {
            ROTATED = Math.toRadians(-90);
            
            backdropCenterPos = new Vector2d(18, -50);
            backdropLeftPos = new Vector2d(23, -50);
            backdropRightPos = new Vector2d(13, -50);

            afterDropCenter = new Vector2d(18, -45);
            afterDropLeft = new Vector2d(23, -45);
            afterDropRight = new Vector2d(13, -45);

            parkInsidePos = new Vector2d(40, -40); // Rotated
            parkOutsidePos = new Vector2d(-6, -40); // Rotated
            parkOutsideTuck = new Vector2d(-6, -58); // Rotated

            crossTrussPos = new Vector2d(0, 44); // Rotated
            pixelStackPosClose = new Vector2d(24, 62); // Rotated
            backIntoPixelPosClose = new Vector2d(24, 65); // Rotated
            backToIntakePixelClose = new Vector2d(24, 67); // Rotated
            cringerBackdropCenterPos = new Vector2d(18, -49); // Rotated


            // Substation Stuff
            beforePixelCrash = new Vector2d(16, 12); // Rotated
            pixelStackPosFar = new Vector2d(45, 15); // Rotated
            backIntoPixelPosFar = new Vector2d(45, 18); // Rotated
            backToIntakePixelFar = new Vector2d(45, 20); // Rotated

            toBackdropFromPixelStack = new Vector2d(43, -84); // Rotated
            betweenSubBackdrop = new Vector2d(15, -94); // Rotated
            subLeftBackdrop = new Vector2d(20, -97); // Rotated
            subCenterBackdrop = new Vector2d(14, -97); // Rotated
            subRightBackdrop = new Vector2d(8, -97); // Rotated

            subParkPos = new Vector2d(37,-90); // Rotated
        }
    }

    // Universal for all modes
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


    // Backdrop Specific
    // Backdrop coordinates for BLUE side
    public Vector2d backdropCenterPos = new Vector2d(18, 50); // Rotated
    public Vector2d backdropLeftPos = new Vector2d(13, 50); // Rotated
    public Vector2d backdropRightPos = new Vector2d(23, 50); // Rotated

    public Vector2d afterDropCenter = new Vector2d(18, 45); // Rotated
    public Vector2d afterDropLeft = new Vector2d(13, 45); // Rotated
    public Vector2d afterDropRight = new Vector2d(23, 45); // Rotated

    // Parking options for BLUE side
    public Vector2d parkInsidePos = new Vector2d(40, 40); // Rotated
    public Vector2d parkOutsidePos = new Vector2d(-6, 40); // Rotated
    public Vector2d parkOutsideTuck = new Vector2d(-6, 58); // Rotated

    // Going to pixel stack
    public Vector2d rotatedStartPos = new Vector2d(-2, 0); // Rotated
    public Vector2d crossTrussPos = new Vector2d(0, -44); // Rotated
    public Vector2d pixelStackPosClose = new Vector2d(24, -62); // Rotated
    public Vector2d backIntoPixelPosClose = new Vector2d(24, -65); // Rotated
    public Vector2d backToIntakePixelClose = new Vector2d(24, -67); // Rotated
    public Vector2d cringerBackdropCenterPos = new Vector2d(18, 49); // Rotated


    // Substation Specific
    // Substation coordinates for BLUE side
    public Vector2d beforePixelCrash = new Vector2d(16, -12); // Rotated
    public Vector2d pixelStackPosFar = new Vector2d(45, -15); // Rotated
    public Vector2d backIntoPixelPosFar = new Vector2d(45, -18); // Rotated
    public Vector2d backToIntakePixelFar = new Vector2d(45, -20); // Rotated

    public Vector2d toBackdropFromPixelStack = new Vector2d(43, 84); // Rotated
    public Vector2d betweenSubBackdrop = new Vector2d(15, 94); // Rotated

    public Vector2d subLeftBackdrop = new Vector2d(8, 97); // Rotated
    public Vector2d subCenterBackdrop = new Vector2d(14, 97); // Rotated
    public Vector2d subRightBackdrop = new Vector2d(20, 97); // Rotated

    public Vector2d subParkPos = new Vector2d(37,90); // Rotated
}
