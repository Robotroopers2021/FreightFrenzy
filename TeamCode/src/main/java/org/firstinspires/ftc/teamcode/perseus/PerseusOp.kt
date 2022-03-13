package org.firstinspires.ftc.teamcode.perseus

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.GamepadUtil.left_trigger_pressed
import org.firstinspires.ftc.teamcode.util.GamepadUtil.right_trigger_pressed
import org.firstinspires.ftc.teamcode.util.math.MathUtil


@Config
@TeleOp
class PerseusOp : OpMode(){
    //drive train motors
    lateinit var fl : DcMotor
    lateinit var fr : DcMotor
    lateinit var bl : DcMotor
    lateinit var br : DcMotor
    //intake-outtake system/duck spinner motors
    lateinit var intake : DcMotor
    lateinit var turret : DcMotor
    lateinit var arm : DcMotor
    lateinit var duck : DcMotor
    //mecanum drivetrain variables
    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0

    fun driveControl() {
        drive = MathUtil.cubicScaling(0.75, -gamepad1.left_stick_y.toDouble()) * 0.85
        strafe = MathUtil.cubicScaling(0.75, gamepad1.left_stick_x.toDouble()) * 0.85
        rotate = gamepad1.right_stick_x.toDouble() * 0.65
        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }

    fun intakeControl() {
        if(gamepad1.right_trigger_pressed) {
            intake.power = 1.0
        } else if(gamepad1.left_trigger_pressed) {
            intake.power = -1.0
        } else if(!gamepad1.right_trigger_pressed && !gamepad1.left_trigger_pressed) {
            intake.power = 0.0
        }
    }

    override fun init() {
        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")

        arm = hardwareMap.get(DcMotor::class.java, "Arm")
        turret = hardwareMap.get(DcMotor::class.java, "Turret")
        duck = hardwareMap.get(DcMotor::class.java, "Duck")
        intake = hardwareMap.get(DcMotor::class.java, "Intake")

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        duck.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        telemetry.addData("STATUS", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        driveControl()
        intakeControl()
    }
}