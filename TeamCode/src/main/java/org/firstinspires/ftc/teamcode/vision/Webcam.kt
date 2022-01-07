package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

class Webcam : Subsystem {
    private lateinit var webcam: OpenCvCamera
    lateinit var pipeline: CupPipeline


    var cupState = CupStates.LEFT
        private set
    enum class CupStates {
        LEFT, MIDDLE, RIGHT
    }



    fun init(hardwareMap: HardwareMap) {
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

    }

    override fun sendDashboardPacket(debugging: Boolean) {

    }

    override fun reset() {
        Globals.CUP_LOCATION = cupState
        webcam.stopStreaming()
    }
}