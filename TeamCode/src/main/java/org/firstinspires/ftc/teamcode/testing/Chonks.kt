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
    lateinit var arm:DcMotor
    lateinit var duckSpinner:DcMotor
    lateinit var hi:DcMotor

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0
    var armpower = 0.4
    var  duck = 0.4
    var hilights= 0.0


    private fun driveControl() {
        drive = (-gamepad1.left_stick_y.toDouble()) * 0.85
        strafe= (gamepad1.left_stick_x.toDouble()) * 0.85
        rotate= (gamepad1.right_stick_x.toDouble()) * 0.85

        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }

   private fun armControl() {
        if (gamepad1.dpad_left) {
            arm.power = armpower
        } else if(gamepad1.dpad_right) {
            arm.power = -armpower
        } else {
           arm.power=0.0
        }
   }

    private fun duckControl() {

        if (gamepad1.dpad_up) {
            duckSpinner.power = duck
        } else if (gamepad1.dpad_down) {
            duckSpinner.power = -duck
        }else {
            duckSpinner.power=0.0
        }
    }
    private fun hiControl() {
        hilights= (gamepad2.right_stick_x.toDouble()) * 0.85
        hi.power=hilights


        }


    override fun init() {

        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")
        arm = hardwareMap.get(DcMotor::class.java, "Arm" )
        duckSpinner = hardwareMap.get(DcMotor::class.java, "DuckL" )
        hi = hardwareMap.get(DcMotor::class.java, "Intake" )

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        duckSpinner.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hi.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE




    }
    override fun loop() {
        driveControl()
        armControl()
        duckControl()
        hiControl()
    }






}