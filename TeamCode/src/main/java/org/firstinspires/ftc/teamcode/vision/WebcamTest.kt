package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.firstinspires.ftc.teamcode.telemetryStuff.Dashboard

class WebcamTest : Subsystem{
    private lateinit var webcam : OpenCvCamera
    private lateinit var pipeline: Pipeline
    var cupState = pipeline.cupState



    fun init() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val webcamName = hardwareMap[WebcamName::class.java, "Webcam"]
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        pipeline = Pipeline()
        webcam.setPipeline(pipeline)

        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {

            }
        })

    }

    override fun sendDashboardPacket(debugging: Boolean) {

    }

    override fun update() {
        cupState
    }

    override fun reset() {
        webcam.stopStreaming()
    }

}