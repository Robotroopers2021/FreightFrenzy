package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.math.MathUtil
import com.qualcomm.hardware.lynx.LynxModule




@Config
@TeleOp
open class OdometryTest : OpMode() {
    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    lateinit var encoderRight : DcMotor
    lateinit var encoderLeft : DcMotor
    lateinit var encoderAux : DcMotor

    var odoPose = Pose2d(0.0,0.0,0.0)

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0

    var TICKS_PER_INCH = 1892.3724

    //constants that define the geometry of the robot
    var trackwidth = MathUtil.cmCalc(5.0)   //distance between encoderRight and encoderLeft in cm
    var B = MathUtil.cmCalc(0.0)   //distance between midpoint of encoderRight and encoderAux in cm
    var R = MathUtil.cmCalc(0.0)   //wheel radius in cm
    var N = 0.0                       //encoder ticks per revolution, Rev encoder
    var inches_per_tick = Math.PI * 2.3622/4000

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
        oldAuxPos   = currentAuxPos

        currentRightPos = (-encoderRight.currentPosition).toDouble()
        currentLeftPos = (-encoderLeft.currentPosition).toDouble()
        currentAuxPos = encoderAux.currentPosition.toDouble()

        val dn1  = currentLeftPos - oldLeftPos
        val dn2 = currentRightPos - oldRightPos
        val dn3 = currentAuxPos - oldAuxPos

        val dtheta = inches_per_tick * (dn1 - dn2) / trackwidth
        val dx = inches_per_tick * (dn1 - dn2) / 2.0
        val dy = inches_per_tick * (dn3 - (dn2 - dn1) * B / trackwidth)

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
        drive = MathUtil.cubicScaling(0.75, gamepad1.left_stick_y.toDouble()) * 0.85
        strafe = MathUtil.cubicScaling(0.75, -gamepad1.left_stick_x.toDouble()) * 0.85
        rotate = -gamepad1.right_stick_x.toDouble() * 0.65
        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }


    private fun telemetry() {
        telemetry.addData("Left", currentLeftPos)
        telemetry.addData("Right", currentRightPos)
        telemetry.addData("Aux", currentAuxPos)
        telemetry.addData("x", odoPose.x)
        telemetry.addData("y",odoPose.y)
        telemetry.addData("heading", odoPose.heading)
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

        var allHubs = hardwareMap.getAll(LynxModule::class.java)

        hardwareMap.getAll(LynxModule::class.java).forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

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