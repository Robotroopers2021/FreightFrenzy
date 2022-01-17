package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.util.GamepadUtil.dpad_up_pressed
import org.firstinspires.ftc.teamcode.util.GamepadUtil.left_trigger_pressed
import org.firstinspires.ftc.teamcode.util.GamepadUtil.right_trigger_pressed
import org.firstinspires.ftc.teamcode.util.math.MathUtil
import kotlin.math.cos

@Config
@TeleOp
class SlowerIntake : OpMode() {
    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    lateinit var intakeMotor: DcMotor
    lateinit var duck: DcMotor
    lateinit var arm: DcMotor
    lateinit var outtakeServo: Servo
    lateinit var distanceSensor: Rev2mDistanceSensor

    private var motionTimer = ElapsedTime()

    var value = 0.0

    var prevTime : Long = 0

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0
    var duckPower = 0.75

    var armController = PIDFController(PIDCoefficients(kp, ki, kd))

    var degreesPerTick = 90 / 184.0
    var ticksPerDegree = 184.0 / 90
    var targetTicks = 0.0
    var output = 0.0
    var pidOutput = 0.0
    var feedForward = 0.0

    private fun moveArmToDegree(degrees: Double) {
        targetAngle = degrees
        targetTicks = targetAngle * ticksPerDegree
        armController.reset()
        armController.targetPosition = targetTicks
    }

    private fun getFeedForward(targetAngle: Double): Double {
        return cos(targetAngle) * kcos
    }

    private fun armControl() {
        when {
            gamepad1.left_bumper -> {
                moveArmToDegree(depositAngle)
            }
            gamepad1.right_bumper -> {
                moveArmToDegree(restAngle)
                outtakeServo.position = 0.90
            }
            gamepad1.a -> {
                moveArmToDegree(sharedAngle)
            }
            gamepad1.x -> {
                moveArmToDegree(sharedAngleAlliance)
            }
            gamepad1.b -> {
                moveArmToDegree(sharedAngleEnemy)
            }
            gamepad1.y -> {
                moveArmToDegree(middlePos)
            }
        }


        val currentPosition = (arm.currentPosition - 114).toDouble()

        feedForward = getFeedForward(Math.toRadians(targetAngle))
        pidOutput = armController.update(currentPosition)
        output = feedForward + pidOutput

        arm.power = output


        val packet = TelemetryPacket()
        packet.put("target angle", targetAngle)
        packet.put("output", output)
        packet.put("Current Position", currentPosition)
        packet.put("degrees", currentPosition * degreesPerTick)
        packet.put("feedforward", getFeedForward(Math.toRadians(targetAngle)))
        packet.put("target ticks", targetTicks)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

    private fun driveControl() {
        drive = MathUtil.cubicScaling(0.75, -gamepad1.left_stick_y.toDouble()) * 0.85
        strafe = MathUtil.cubicScaling(0.75, gamepad1.left_stick_x.toDouble()) * 0.85
        rotate = MathUtil.cubicScaling(0.85, gamepad1.right_stick_x.toDouble()) * 0.65
        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }


    private fun stopIntake() {
        intakeMotor.power = 0.0
    }

    private fun startIntake() {
        intakeMotor.power = -1.0
    }

    private fun openIndexer() {
        outtakeServo.position = 0.90
    }

    private fun lockIndexer() {
        outtakeServo.position = 0.80
    }

    private enum class IntakeSequenceStates {
        INTAKE_OUTTAKE_RESET,
        INTAKE,
        STOP_AND_LOCK
    }

    private val intakeSequence = StateMachineBuilder<IntakeSequenceStates>()
        .state(IntakeSequenceStates.INTAKE_OUTTAKE_RESET)
        .onEnter {
            openIndexer()
        }
        .transitionTimed(0.25)
        .state(IntakeSequenceStates.INTAKE)
        .onEnter {
            startIntake()
        }
        .transition {
            value <= 3.0
        }
        .state(IntakeSequenceStates.STOP_AND_LOCK)
        .onEnter {
            stopIntake()
            lockIndexer()
        }

        .build()

    private fun intakeControl() {
        if (gamepad1.right_trigger > 0.5) {
            intakeMotor.power = 1.0
        }

        if (!gamepad1.right_trigger_pressed && !gamepad1.left_trigger_pressed) {
            intakeMotor.power = 0.0
        }

        if (gamepad1.left_trigger_pressed && !intakeSequence.running && value > 3) {
            intakeSequence.start()
        }

        if (!gamepad1.left_trigger_pressed && intakeSequence.running) {
            intakeSequence.stop()
            intakeSequence.reset()
        }

        if (intakeSequence.running && gamepad1.left_trigger_pressed) {
            intakeSequence.update()
        }
    }


    private enum class DuckSpinnerStates {
        RUN_SLOW,
        RUN_FAST,
        STOP
    }

    private val duckSpinnerSequence = StateMachineBuilder<DuckSpinnerStates>()
        .state(DuckSpinnerStates.RUN_SLOW)
        .onEnter {
            duck.power = 0.5
        }
        .transitionTimed(1.0)
        .state(DuckSpinnerStates.RUN_FAST)
        .onEnter {
            duck.power = 0.85
        }
        .transitionTimed(1.2)
        .state(DuckSpinnerStates.STOP)
        .onEnter {
            duck.power = 0.0
        }

        .build()

    private fun duckSpinnerSequenceStart() {
        if (gamepad1.dpad_left) {
            duck.power = -duckPower
        }
        if (gamepad1.dpad_right) {
            duck.power = duckPower
        }
        if (gamepad1.dpad_up_pressed && !duckSpinnerSequence.running) {
            duckSpinnerSequence.start()
        }
        if (!gamepad1.dpad_up_pressed && duckSpinnerSequence.running) {
            duckSpinnerSequence.stop()
            duckSpinnerSequence.reset()
        }
        if (duckSpinnerSequence.running && gamepad1.dpad_up_pressed) {
                duckSpinnerSequence.update()
        }

    }

    private fun outtakeControl() {
        if (gamepad2.a) {
            outtakeServo.position = 0.90
        }
        if (gamepad2.b) {
            outtakeServo.position = 0.6
        }
        if (gamepad2.x) {
            outtakeServo.position = 0.80
        }
    }


    private fun distanceSensorControl() {
        if (value <= 3) {
            gamepad1.rumble(750)
        } else {
            gamepad1.stopRumble()
        }
    }

    private fun telemetry() {
        telemetry.addData("dsensor", value)
    }



    override fun init() {
        //Connect Motor
        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")
        arm = hardwareMap.dcMotor["Arm"]
        duck = hardwareMap.get(DcMotor::class.java, "DuckL")
        distanceSensor = hardwareMap.get(
            Rev2mDistanceSensor::class.java,
            "distanceSensor"
        ) as Rev2mDistanceSensor
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        outtakeServo.position = 0.90
        armController.reset()
        targetAngle = restAngle
        targetTicks = restAngle * ticksPerDegree
        armController.targetPosition = targetTicks
        telemetry.addData("STATUS", "Initialized")
        telemetry.update()
    }


    override fun loop() {
        driveControl()
        armControl()
        intakeControl()
        outtakeControl()
        //distanceSensorControl()
        duckSpinnerSequenceStart()
        telemetry()
        value = distanceSensor.getDistance(DistanceUnit.INCH)
        telemetry.addData("loop time", System.currentTimeMillis()-prevTime)
        prevTime = System.currentTimeMillis()
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
