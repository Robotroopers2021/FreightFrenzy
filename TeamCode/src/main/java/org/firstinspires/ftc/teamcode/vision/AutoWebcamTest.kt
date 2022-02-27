package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.testing.PipelineTesting
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

class AutoWebcamTest : Subsystem{
    private lateinit var webcam : OpenCvCamera
    lateinit var pipeline : PipelineTesting
    var armpos : Int = 0


    fun init(hardwareMap: HardwareMap) {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val webcamName = hardwareMap[WebcamName::class.java, "Webcam"]
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)
        pipeline = PipelineTesting()
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

    fun takePicture() {
        armpos = (pipeline.storedx/10)
    }

    fun stopWebcam() {
        webcam.pauseViewport()
    }

    override fun update() {


    }

    override fun reset() {
        webcam.stopStreaming()
    }

}