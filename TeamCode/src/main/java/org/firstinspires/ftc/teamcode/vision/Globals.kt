package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.vision.Webcam
import org.firstinspires.ftc.teamcode.vision.AllianceSide

@Config
object Globals {
    var ALLIANCE_SIDE = AllianceSide.BLUE
    var IS_AUTO = false
    var CUP_LOCATION = Webcam.CupStates.LEFT
}