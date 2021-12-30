package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.firstinspires.ftc.teamcode.vision.Globals
import org.firstinspires.ftc.teamcode.telemetryStuff.Dashboard
import org.firstinspires.ftc.teamcode.vision.Subsystem

class Webcam : Subsystem {
    private lateinit var webcam: OpenCvCamera
    private lateinit var pipeline: CupPipeline

    var cupState = CupStates.LEFT
        private set
    enum class CupStates {
        LEFT, MIDDLE, RIGHT
    }

    fun init() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val webcamName = hardwareMap[WebcamName::class.java, "Webcam"]
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        pipeline = CupPipeline()
        webcam.setPipeline(pipeline)

        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {

            }
        })
    }

    override fun update() {
        val area = pipeline.getRectArea()
        val midpoint = pipeline.getRectMidpointX()

        cupState = when {
            area < CupPipeline.MIN_AREA -> CupStates.LEFT
            midpoint > 200 -> CupStates.RIGHT
            else -> CupStates.MIDDLE
        }

        Dashboard["cup state"] = cupState
        Dashboard["midpoint"] = midpoint
    }

    override fun sendDashboardPacket(debugging: Boolean) {

    }

    override fun reset() {
        Globals.CUP_LOCATION = cupState
        webcam.stopStreaming()
    }
}