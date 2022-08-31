package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Config
@TeleOp
open class Chonks : OpMode() {
    lateinit var fl:DcMotor
    lateinit var fr:DcMotor
    lateinit var bl:DcMotor
    lateinit var br:DcMotor

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0

    private fun driveControl() {
        drive = (-gamepad1.left_stick_y.toDouble()) * 0.85
        strafe= (-gamepad1.left_stick_x.toDouble()) * 0.85
        rotate= (-gamepad1.right_stick_x.toDouble()) * 0.85
    }

    override fun init() {

        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE




    }
    override fun loop() {
        driveControl()
    }






}