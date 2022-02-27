package org.firstinspires.ftc.teamcode.testing

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class PipelineTesting : OpenCvPipeline() {

    var autoMatrix = Mat()
    var grayMatrix = Mat()
    val total : Double = 0.0
    var storedx = 0
    var storedy = 0

    override fun processFrame(input: Mat): Mat {

        input.copyTo(autoMatrix)

        if (autoMatrix.empty()) {
            return input
        }

        Imgproc.cvtColor(autoMatrix, grayMatrix, Imgproc.COLOR_RGB2HSV)

        for(i in 320 downTo 2 step 5) {
            for(j in 240 downTo 120 step 5) {
                val fmat = grayMatrix.submat(i-1, i+0, j-1, j+0)
                val total = Core.mean(fmat).`val`[2]
                if (total >= 200) {
                    storedx = i
                    storedy = j
                    break
                }
            }
        }
        return grayMatrix
    }
}