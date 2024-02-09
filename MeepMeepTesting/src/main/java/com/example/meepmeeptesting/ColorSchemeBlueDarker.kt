package com.example.meepmeeptesting

import com.noahbres.meepmeep.core.colorscheme.ColorManager
import com.noahbres.meepmeep.core.colorscheme.ColorScheme
import java.awt.Color

open class ColorSchemeBlueDarker : ColorScheme() {
    override val isDark: Boolean = true

    override val BOT_BODY_COLOR = ColorManager.COLOR_PALETTE.BLUE_700
    override val BOT_WHEEL_COLOR = ColorManager.COLOR_PALETTE.BLUE_900
    override val BOT_DIRECTION_COLOR = ColorManager.COLOR_PALETTE.BLUE_900

    override val AXIS_X_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_900
    override val AXIS_Y_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_900

    override val TRAJCETORY_PATH_COLOR: Color = ColorManager.COLOR_PALETTE.RED_700
    override val TRAJECTORY_TURN_COLOR: Color = ColorManager.COLOR_PALETTE.ORANGE_700
    override val TRAJECTORY_MARKER_COLOR: Color = ColorManager.COLOR_PALETTE.GREEN_700


    override val AXIS_NORMAL_OPACITY: Double = 0.2
    override val AXIS_HOVER_OPACITY: Double = 0.8

    override val TRAJECTORY_SLIDER_BG: Color = ColorManager.COLOR_PALETTE.GRAY_200
    override val TRAJECTORY_SLIDER_FG: Color = ColorManager.COLOR_PALETTE.BLUE_700
    override val TRAJECTORY_TEXT_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_900

    override val UI_MAIN_BG: Color = ColorManager.COLOR_PALETTE.GRAY_200
}