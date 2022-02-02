package org.firstinspires.ftc.teamcode.archived.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.util.GamepadUtil.dpad_up_pressed
import org.firstinspires.ftc.teamcode.util.GamepadUtil.left_trigger_pressed
import org.firstinspires.ftc.teamcode.util.GamepadUtil.right_trigger_pressed
import org.firstinspires.ftc.teamcode.util.math.MathUtil
import java.util.concurrent.TimeUnit
import kotlin.math.cos

@Config
@TeleOp
open class Odometrytesting : OpMode() {
    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    lateinit var encoderRight : DcMotor
    lateinit var encoderLeft : DcMotor
    lateinit var encoderAux : DcMotor

    lateinit var odoPose : Pose2d

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0

    //constants that define the geometry of the robot
    var L = MathUtil.cmCalc(0.0)   //distance between encoderRight and encoderLeft in cm
    var B = MathUtil.cmCalc(0.0)   //distance between midpoint of encoderRight and encoderAux in cm
    var R = MathUtil.cmCalc(0.0)   //wheel radius in cm
    var N = 0.0                       //encoder ticks per revolution, Rev encoder
    var cm_per_tick = 2.0 * Math.PI * R/N

    //odometry update variables
    var currentRightPos = 0.0
    var currentLeftPos = 0.0
    var currentAuxPos = 0.0

    var oldRightPos = 0.0
    var oldLeftPos = 0.0
    var oldAuxPos = 0.0

    fun odometry() {
        oldRightPos = currentRightPos
        oldLeftPos = currentLeftPos
        oldAuxPos = currentAuxPos

        currentRightPos = (-encoderRight.currentPosition).toDouble()
        currentLeftPos = (-encoderLeft.currentPosition).toDouble()
        currentAuxPos = encoderAux.currentPosition.toDouble()

        val dn1  = currentLeftPos - oldLeftPos
        val dn2 = currentRightPos - oldRightPos
        val dn3 = currentAuxPos - oldAuxPos

        val dtheta = cm_per_tick * (dn1 - dn2) / L
        val dx = cm_per_tick * (dn1 - dn2) / 2.0
        val dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L)

        val theta = odoPose.heading + (dtheta / 2.0)
        val robotx = dx * Math.cos(theta) - dy * Math.sin(theta)
        val roboty = dx * Math.sin(theta) + dy * Math.cos(theta)
        val roboth = dtheta
        
//         odoPose = Pose2d(robotx, roboty, roboth)
        val newX = odoPose.x + robotx
        val newY = odoPose.y + roboty
        val newH = odoPose.heading + roboth

        odoPose = Pose2d(newX, newY, newH)

    }

    private fun driveControl() {
        drive = MathUtil.cubicScaling(0.75, -gamepad1.left_stick_y.toDouble()) * 0.85
        strafe = MathUtil.cubicScaling(0.75, gamepad1.left_stick_x.toDouble()) * 0.85
        rotate = gamepad1.right_stick_x.toDouble() * 0.65
        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }


    private fun telemetry() {
        telemetry.addData("Left", currentLeftPos)
        telemetry.addData("Right", currentRightPos)
        telemetry.addData("Angle", currentAuxPos)
    }

    override fun init() {
        //Connect Motor
        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.mode = (DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        fr.mode = (DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        bl.mode = (DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        br.mode = (DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        encoderRight = fr
        encoderLeft = fl
        encoderAux = br

        telemetry.addData("STATUS", "Initialized")
        telemetry.update()
    }


    override fun loop() {
        driveControl()
        odometry()
        telemetry()
    }

    companion object {
        @JvmStatic var kp = 0.015
        @JvmStatic var ki = 0.0
        @JvmStatic var kd = 0.00075
        @JvmStatic var targetAngle = 0.0
        @JvmStatic var kcos = 0.275
        @JvmStatic var kv = 0.0
        @JvmStatic var depositAngle = 94.0
        @JvmStatic var restAngle = -55.0
        @JvmStatic var sharedAngle = 172.0
        @JvmStatic var sharedAngleAlliance = 178.0
        @JvmStatic var sharedAngleEnemy = 164.0
        @JvmStatic var middlePos = 134.0
    }
}
