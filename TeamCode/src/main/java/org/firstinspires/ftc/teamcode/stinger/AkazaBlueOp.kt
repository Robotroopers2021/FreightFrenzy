package org.firstinspires.ftc.teamcode.stinger

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
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
import kotlin.math.pow

@Config
@TeleOp
open class AkazaBlueOp : OpMode() {
    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    lateinit var intakeMotor: DcMotor
    lateinit var duck: DcMotor
    lateinit var arm: DcMotor
    lateinit var outtakeServo: Servo
    lateinit var capServo : Servo
    lateinit var distanceSensor: Rev2mDistanceSensor

    lateinit var blinkinLedDriver: RevBlinkinLedDriver
    lateinit var pattern: BlinkinPattern

    private var motionTimer = ElapsedTime()
    private var ledTimer = ElapsedTime()

    private var capUp = 0.1
    private var capDown = -0.1


    private val LED_PERIOD = 90.0

    lateinit var displayKind: DisplayKind
    lateinit var ledCycleDeadline: Deadline

    var value = 0.0

    var prevTime : Long = 0

    var capPos = 1.0

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0

    var armController = PIDFController(PIDCoefficients(kp, ki, kd))

    var degreesPerTick = 90 / 184.0
    var ticksPerDegree = 184.0 / 90
    var targetTicks = 0.0
    var output = 0.0
    var pidOutput = 0.0
    var feedForward = 0.0

    enum class DisplayKind {
        AUTO
    }

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
                lockIndexer()
                moveArmToDegree(depositAngle)
            }
            gamepad1.right_bumper -> {
                moveArmToDegree(restAngle)
                lockIndexer()
                stopIntake()
            }
            gamepad1.a -> {
                moveArmToDegree(sharedAngle)
            }
            gamepad1.x -> {
                moveArmToDegree(middlePosTwo)
            }
            gamepad1.b -> {
                moveArmToDegree(sharedAngleEnemy)
            }
            gamepad1.y -> {
                moveArmToDegree(middlePos)
            }
            gamepad2.y-> {
                moveArmToDegree(-250.0)
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
        rotate = gamepad1.right_stick_x.toDouble() * 0.65
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

    private fun reverseIntake(){
        intakeMotor.power = 1.0
    }

    private fun openIndexer() {
        outtakeServo.position = 0.90
    }

    private fun lockIndexer() {
        outtakeServo.position = 0.78
    }

    private enum class IntakeSequenceStates {
        INTAKE_OUTTAKE_RESET,
        WAIT,
        STOP_AND_LOCK,
        INTAKE_REVERSE,
        INTAKE_OFF
    }

    private val intakeSequence = StateMachineBuilder<IntakeSequenceStates>()
        .state(IntakeSequenceStates.INTAKE_OUTTAKE_RESET)
        .onEnter {
            openIndexer()
            startIntake()
        }
        .transition {
            value <= 75.0
        }
        .state(IntakeSequenceStates.WAIT)
        .onEnter{}
        .transitionTimed(0.2)
        .state(IntakeSequenceStates.STOP_AND_LOCK)
        .onEnter {
            stopIntake()
            lockIndexer()
        }
        .transitionTimed(0.01)
        .state(IntakeSequenceStates.INTAKE_REVERSE)
        .onEnter{
            reverseIntake()
        }
        .transitionTimed(0.5)
        .state(IntakeSequenceStates.INTAKE_OFF)
        .onEnter{
            stopIntake()
        }

        .build()

    private fun intakeControl() {
        if (gamepad1.right_trigger > 0.5) {
            intakeMotor.power = 1.0
        }

        if (!gamepad1.right_trigger_pressed && !gamepad1.left_trigger_pressed) {
            intakeMotor.power = 0.0
        }

        if (gamepad1.left_trigger_pressed && !intakeSequence.running && value > 75.0) {
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
        YEET,
        STOP
    }

    private val duckSpinnerSequence = StateMachineBuilder<DuckSpinnerStates>()
        .state(DuckSpinnerStates.RUN_SLOW)
        .onEnter {
            duck.power = 0.25
        }
        .transitionTimed(0.5)
        .state(DuckSpinnerStates.RUN_FAST)
        .onEnter {
            duck.power = 0.35
        }
        .transitionTimed(0.5)
        .state(DuckSpinnerStates.YEET)
        .onEnter{
            duck.power = 0.85
        }
        .transitionTimed(0.4)
        .state(DuckSpinnerStates.STOP)
        .onEnter {
            duck.power = 0.0
        }

        .build()

    private fun duckSpinnerSequenceStart() {
        if (gamepad1.dpad_up_pressed && !duckSpinnerSequence.running) {
            duckSpinnerSequence.start()
        }
        if (!gamepad1.dpad_up_pressed && duckSpinnerSequence.running) {
            duckSpinnerSequence.stop()
            duckSpinnerSequence.reset()
            motionTimer.reset()
        }
        if (!gamepad1.dpad_up_pressed) {
            duck.power = 0.0
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
            outtakeServo.position = 0.78
        }
    }

    private fun capControl() {
        if(gamepad2.dpad_up) {
            capPos = capPos + 0.01
        } else if(gamepad2.dpad_down) {
            capPos = capPos - 0.01
        }

        if(gamepad2.left_bumper) {
            //pickup pos
            capPos = 0.0
        }

        if(gamepad2.left_trigger_pressed) {
            //ready to drop pos
            capPos = 0.6
        }

        if(gamepad2.right_trigger_pressed) {
            //rest pos
            capPos = 1.0

        }

        capServo.position = capPos
    }

    private fun telemetry() {
        telemetry.addData("dsensor", value)
        telemetry.addData("loop time", System.currentTimeMillis()-prevTime)
        prevTime = System.currentTimeMillis()
    }
    private fun doAutoDisplay() {
        if (ledCycleDeadline.hasExpired()) {
            pattern = BlinkinPattern.FIRE_LARGE
            blinkinLedDriver.setPattern(pattern)
            ledCycleDeadline.reset()
        }
    }
    private fun BlinkBlink() {
        if (displayKind == DisplayKind.AUTO) {
            doAutoDisplay()
        }
        if ((outtakeServo.position > 0.75 && outtakeServo.position < 0.82) && ledTimer.seconds() < LED_PERIOD) {
            pattern = BlinkinPattern.LIGHT_CHASE_BLUE
            blinkinLedDriver.setPattern(pattern)
        } else if((outtakeServo.position < 0.75 || outtakeServo.position > 0.82) && ledTimer.seconds() < LED_PERIOD){
            pattern = BlinkinPattern.DARK_RED
            blinkinLedDriver.setPattern(pattern)
        } else if (gamepad1.right_bumper && ledTimer.seconds() < LED_PERIOD &&
            (outtakeServo.position > 0.75 && outtakeServo.position < 0.82)) {
            pattern = BlinkinPattern.DARK_RED
        }
    }

    private fun getValue() {
        value = distanceSensor.getDistance(DistanceUnit.MM)
    }

    override fun init() {
        //Connect Motor
        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")
        arm = hardwareMap.dcMotor["Arm"]
        duck = hardwareMap.get(DcMotor::class.java, "DuckL")
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") as Rev2mDistanceSensor
        intakeMotor = hardwareMap.dcMotor["Intake"]
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        outtakeServo = hardwareMap.get(Servo::class.java, "Outtake") as Servo
        capServo = hardwareMap.get(Servo::class.java, "Cap") as Servo
        capServo.position = 1.0

        displayKind = DisplayKind.AUTO

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin")
        pattern = BlinkinPattern.DARK_RED
        blinkinLedDriver.setPattern(pattern)

        ledCycleDeadline = Deadline(LED_PERIOD.toLong(), TimeUnit.SECONDS)

        duck.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

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
        duckSpinnerSequenceStart()
        BlinkBlink()
        getValue()
        telemetry()
        capControl()
    }

    companion object {
        @JvmStatic var kp = 0.015
        @JvmStatic var ki = 0.0
        @JvmStatic var kd = 0.00075
        @JvmStatic var targetAngle = 0.0
        @JvmStatic var kcos = 0.275
        @JvmStatic var depositAngle = 94.0
        @JvmStatic var restAngle = -55.0
        @JvmStatic var sharedAngle = 172.0
        @JvmStatic var middlePosTwo = 136.0
        @JvmStatic var sharedAngleEnemy = 164.0
        @JvmStatic var middlePos = 139.0
    }
}
