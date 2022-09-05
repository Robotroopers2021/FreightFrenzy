package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.java_websocket.handshake.HandshakeImpl1Client

@Config
@TeleOp
class Chonks : OpMode() {
    lateinit var fl:DcMotor
    lateinit var fr:DcMotor
    lateinit var bl:DcMotor
    lateinit var br:DcMotor

    var drive=0.0
    var rotate=0.0
    var strafe=0.0

    private fun driveControl() {
        drive = (-gamepad1.left_stick_y.toDouble()) * 0.85
        rotate = (-gamepad1.left_stick_x.toDouble()) * 0.85
        strafe = (-gamepad1.left_stick_x.toDouble()) * 0.85
        fl.power = drive + strafe + rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
        fr.power = drive - strafe - rotate

    }

    override fun init() {
        driveControl()
    }

    override fun loop() {
        fl = BlocksOpModeCompanion.hardwareMap.get(DcMotor::class.java, "FL")
        fr = BlocksOpModeCompanion.hardwareMap.get(DcMotor::class.java, "FR")
        br = BlocksOpModeCompanion.hardwareMap.get(DcMotor::class.java, "BR")
        bl = BlocksOpModeCompanion.hardwareMap.get(DcMotor::class.java, "BL")

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE
    }


}