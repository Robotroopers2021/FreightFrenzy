package org.firstinspires.ftc.teamcode.util
import com.qualcomm.robotcore.hardware.Gamepad

object GamepadUtil {
    val Gamepad.left_trigger_pressed
        get() = left_trigger > 0.5

    val Gamepad.right_trigger_pressed
        get() = right_trigger > 0.5
}