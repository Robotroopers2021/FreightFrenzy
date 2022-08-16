package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
@TeleOp
@Disabled
class Chonks {

    //Declaring my motors, sevos, and sensors
    lateinit var intakeMotor: DcMotor
    lateinit var servo: Servo
    lateinit var sensor : Rev2mDistanceSensor

    private var value = 0.0

    private var intakeControl = 0.0

    private fun intakeControl() {
        if (gamepad1.a) {
            intakeMotor.power = 1.0
        }
        if (gamepad1.b) {
            intakeControl = 1.0
        }
        if (gamepad2.x) {
            servo.position = 0.5
        }
        if (gamepad2.y) {
            sensor.getDistance(DistanceUnit.MM)
        }
        if (value < 25) {
            intakeMotor.power = 0.0
    }

    fun getDistance() {
        value = sensor.getDistance(DistanceUnit.MM)
        }
    }
}
