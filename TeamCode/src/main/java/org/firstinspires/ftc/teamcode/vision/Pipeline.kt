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
    enum class CupStates {
        LEFT, CENTER, RIGHT, NOT_WORKING
    }

    var MIN_R = 230


    override fun processFrame (input : Mat) : Mat {
        input.copyTo(workingMatrix)

        if (workingMatrix.empty()) {
            return input
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2RGBA)

        val matLeft = workingMatrix.submat(120, 150, 10, 50)
        val matCenter = workingMatrix.submat(120, 150, 150, 190)
        val matRight = workingMatrix.submat(120, 150, 290, 320)

        Imgproc.rectangle(workingMatrix, Rect(10, 120, 40, 30), Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(workingMatrix, Rect(150, 120, 40, 30), Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(workingMatrix, Rect(290, 120, 40, 30), Scalar(0.0, 255.0, 0.0))

        val LeftTotal = Core.sumElems(matLeft).`val`[0]
        val CenterTotal = Core.sumElems(matCenter).`val`[0]
        val RightTotal = Core.sumElems(matRight).`val`[0]

        if (LeftTotal < MIN_R && RightTotal < MIN_R && CenterTotal < MIN_R) {
            //Left is TSE
            cupState = CupStates.LEFT
        }

        else if ( LeftTotal < MIN_R && CenterTotal > RightTotal) {
            //Center is TSE
            cupState = CupStates.CENTER
        }

        else if ( RightTotal > CenterTotal && LeftTotal < MIN_R) {
            //Right is TSE
            cupState = CupStates.RIGHT
        }

        else {
            cupState = CupStates.NOT_WORKING
        }

        return workingMatrix
    }
}