package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

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

        fl = hardwareMap.get(DcMotor::class.java, "frontLeft")
        fr = hardwareMap.get(DcMotor::class.java, "frontRight")
        bl = hardwareMap.get(DcMotor::class.java, "backLeft")
        br = hardwareMap.get(DcMotor::class.java, "backRight")

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE




    }
    override fun loop() {
        driveControl()
    }






}