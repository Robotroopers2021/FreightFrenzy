package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class Pipeline : OpenCvPipeline(){

    private var workingMatrix = Mat()

    var cupState = CupStates.LEFT
        private set
    enum class CupStates {
        LEFT, CENTER, RIGHT
    }


    override fun processFrame (input : Mat) : Mat {
        input.copyTo(workingMatrix)

        if (workingMatrix.empty()) {
            return input
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb)

        val matLeft = workingMatrix.submat(120, 150, 10, 50)
        val matCenter = workingMatrix.submat(120, 150, 80, 120)
        val matRight = workingMatrix.submat(120, 150, 150, 190)

        Imgproc.rectangle(workingMatrix, Rect(10, 120, 40, 30), Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(workingMatrix, Rect(80, 120, 40, 30), Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(workingMatrix, Rect(150, 120, 40, 30), Scalar(0.0, 255.0, 0.0))

        val LeftTotal = Core.sumElems(matLeft).`val`[2]
        val CenterTotal = Core.sumElems(matCenter).`val`[2]
        val RightTotal = Core.sumElems(matRight).`val`[2]

        if (LeftTotal > CenterTotal && LeftTotal > RightTotal) {
            //Left is TSE
            cupState = CupStates.LEFT
        }

        if ( CenterTotal > LeftTotal && CenterTotal > RightTotal) {
            //Center is TSE
            cupState = CupStates.CENTER
        }

        if ( RightTotal > CenterTotal && RightTotal > LeftTotal) {
            //Right is TSE
            cupState = CupStates.RIGHT
        }

        return workingMatrix
    }
}