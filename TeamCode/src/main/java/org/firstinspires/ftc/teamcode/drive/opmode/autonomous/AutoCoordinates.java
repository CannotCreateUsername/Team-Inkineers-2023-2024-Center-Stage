package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public class AutoCoordinates {
    public double ROTATED = Math.toRadians(90);
    public double ROTATED_CRINGE = Math.toRadians(92);
    public double STRAIGHT = Math.toRadians(0);

    public double LEFT_PROP = Math.toRadians(45);
    public double RIGHT_PROP = Math.toRadians(-45);

    public double ROTATED_AF = Math.toRadians(135); // April Tag Orientation Far side
    public double ROTATED_AC = Math.toRadians(45); // April Tag Orientation Close side

    public AutoCoordinates(boolean isRedSide, boolean isFarSide) {
        if (isRedSide) {
            ROTATED = Math.toRadians(-90);
            ROTATED_CRINGE = Math.toRadians(-92);
            ROTATED_AF = Math.toRadians(-135);
            ROTATED_AC = Math.toRadians(-45);

            LEFT_PROP = Math.toRadians(-45);
            RIGHT_PROP = Math.toRadians(45);

            if (isFarSide) {
                propLeftPos = new Vector2d(32, 12.5);
                propRightPos = new Vector2d(29, -12);
            }
            backdropCenterPos = new Vector2d(18, -52);
            backdropLeftPos = new Vector2d(24, -52);
            backdropRightPos = new Vector2d(13, -52);

            afterDropCenter = new Vector2d(18, -45);
            afterDropLeft = new Vector2d(23, -45);
            afterDropRight = new Vector2d(13, -45);

            parkInsidePos = new Vector2d(40, -40); // Rotated
            parkOutsidePos = new Vector2d(-6, -40); // Rotated
            parkOutsideTuck = new Vector2d(-6, -58); // Rotated

            crossTrussPos = new Vector2d(0, 44); // Rotated
            pixelStackPosClose = new Vector2d(23, 62); // Rotated
            backIntoPixelPosClose = new Vector2d(23, 65); // Rotated
            backToIntakePixelClose = new Vector2d(23, 67); // Rotated
            cringerBackdropCenterPos = new Vector2d(18, -51); // Rotated
            cringerBackdropPosNotCenter = new Vector2d(13, -51); // Rotated

            // Substation Stuff
            beforePixelCrash = new Vector2d(16, 12); // Rotated
            pixelStackPosFar = new Vector2d(45, 15); // Rotated
            backIntoPixelPosFar = new Vector2d(45, 18); // Rotated
            backToIntakePixelFar = new Vector2d(45, 20); // Rotated

            toBackdropFromPixelStack = new Vector2d(40, -84); // Rotated
            toBackdropFromPixelStackSpline = new Vector2d(32, -78); // Rotated
            betweenSubBackdrop = new Vector2d(15, -94); // Rotated
            subLeftBackdrop = new Vector2d(20, -99); // Rotated
            subCenterBackdrop = new Vector2d(14.5, -99); // Rotated
            subRightBackdrop = new Vector2d(8, -99); // Rotated

            subParkPos = new Vector2d(37,-90); // Rotated
        } else {
            if (isFarSide) {
                propRightPos = new Vector2d(32, -12); // Straight
            }
        }
    }

    // Universal for all modes
    // Starting position
    public Pose2d startPos = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

    // Team prop exact coordinates
    public Vector2d propCenterPos = new Vector2d(32, 0); // Straight
    public Vector2d propLeftPos = new Vector2d(29, 12); // Straight
    public Vector2d propRightPos = new Vector2d(32, -12.5); // Straight

    // Around the team prop for side randomization
    public Vector2d betweenSideProp = new Vector2d(27, 0); // Run between the spike marks to strafe Straight
    public Vector2d backFromCenterProp = new Vector2d(25, 0); // Scoot back 8 inches from center prop Straight
    public Vector2d backFromLeftProp = new Vector2d(25, 12); // Scoot back 4 inches from left prop Straight
    public Vector2d backFromRightProp = new Vector2d(25, -12); // Scoot back 4 inches from right prop Straight


    // Backdrop Specific
    // Backdrop coordinates for BLUE side
    public Vector2d backdropCenterPos = new Vector2d(18, 52); // Rotated
    public Vector2d backdropLeftPos = new Vector2d(13, 52); // Rotated
    public Vector2d backdropRightPos = new Vector2d(24, 52); // Rotated

    public Vector2d afterDropCenter = new Vector2d(18, 45); // Rotated
    public Vector2d afterDropLeft = new Vector2d(13, 45); // Rotated
    public Vector2d afterDropRight = new Vector2d(23, 45); // Rotated

    // Parking options for BLUE side
    public Vector2d parkInsidePos = new Vector2d(40, 40); // Rotated
    public Vector2d parkOutsidePos = new Vector2d(-6, 40); // Rotated
    public Vector2d parkOutsideTuck = new Vector2d(-6, 58); // Rotated

    // Going to pixel stack
    public Vector2d rotatedStartPos = new Vector2d(-4, 0); // Rotated
    public Vector2d crossTrussPos = new Vector2d(-2, -44); // Rotated
    public Vector2d pixelStackPosClose = new Vector2d(23, -62); // Rotated
    public Vector2d backIntoPixelPosClose = new Vector2d(23, -65); // Rotated
    public Vector2d backToIntakePixelClose = new Vector2d(23, -65); // Rotated
    public Vector2d cringerBackdropCenterPos = new Vector2d(18, 51); // Rotated
    public Vector2d cringerBackdropPosNotCenter = new Vector2d(13, 51); // Rotated


    // Substation Specific
    // Substation coordinates for BLUE side
    public Vector2d beforePixelCrash = new Vector2d(16, -12); // Rotated
    public Vector2d pixelStackPosFar = new Vector2d(45, -15); // Rotated
    public Vector2d backIntoPixelPosFar = new Vector2d(45, -18); // Rotated
    public Vector2d backToIntakePixelFar = new Vector2d(45, -20); // Rotated

    public Vector2d toBackdropFromPixelStack = new Vector2d(40, 84); // Rotated
    public Vector2d toBackdropFromPixelStackSpline = new Vector2d(32, 78); // Rotated
    public Vector2d betweenSubBackdrop = new Vector2d(15, 94); // Rotated

    public Vector2d subLeftBackdrop = new Vector2d(8, 99); // Rotated
    public Vector2d subCenterBackdrop = new Vector2d(14.5, 99); // Rotated
    public Vector2d subRightBackdrop = new Vector2d(21, 99); // Rotated

    public Vector2d subParkPos = new Vector2d(37,90); // Rotated
}
