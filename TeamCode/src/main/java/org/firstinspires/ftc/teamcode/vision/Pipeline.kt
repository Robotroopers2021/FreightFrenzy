package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class Pipeline : OpenCvPipeline(){

    var workingMatrix = Mat()

    var cupState = CupStates.LEFT
    enum class CupStates {
        LEFT, CENTER, RIGHT, NOT_WORKING
    }

    var MIN_R = 100

    val matLeft = workingMatrix.submat(120, 150, 10, 50)
    val matCenter = workingMatrix.submat(120, 150, 150, 190)
    val matRight = workingMatrix.submat(120, 150, 290, 320)


    val leftTotal = Core.mean(matLeft).`val`[2]
    val centerTotal = Core.mean(matCenter).`val`[2]
    val rightTotal = Core.mean(matRight).`val`[2]


    override fun processFrame (input : Mat) : Mat {
        input.copyTo(workingMatrix)

        if (workingMatrix.empty()) {
            return input
        }

        val MatLeft = matLeft
        val MatRight = matRight
        val MatCenter = matCenter

        val LeftTotal = leftTotal
        val CenterTotal = centerTotal
        val RightTotal = rightTotal

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb)

        Imgproc.rectangle(workingMatrix, Rect(10, 120, 40, 30), Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(workingMatrix, Rect(150, 120, 40, 30), Scalar(0.0, 255.0, 0.0))
        Imgproc.rectangle(workingMatrix, Rect(290, 120, 40, 30), Scalar(0.0, 255.0, 0.0))

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