package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import org.firstinspires.ftc.teamcode.util.Extensions.d

class CupPipeline : OpenCvPipeline(){

    private var loopcounter = 0
    private var ploopcounter = 0

    private val mat = Mat()
    private val processed = Mat()

    private var maxRect = Rect()

    private var maxArea = 0.0
    private var first = false

    override fun processFrame(input: Mat): Mat {
        val output = input.clone()

        try {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2RGBA)
            Core.inRange(mat, LOWER_LIMIT, UPPER_LIMIT, processed)
            // Core.bitwise_and(input, input, output, processed);

            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, Mat())
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, Mat())
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, Size(5.0, 15.0), 0.00)
            // Find Contours
            val contours: List<MatOfPoint> = ArrayList()
            Imgproc.findContours(processed, contours, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)

            // Draw Contours
//                if (DISPLAY) Imgproc.drawContours(output, contours, -1, DISPLAY_COLOR)

            for (contour in contours) {
                val contourArray = contour.toArray()

                // Bound Rectangle if Contour is Large Enough
                if (contourArray.size >= 15) {
                    val areaPoints = MatOfPoint2f(*contourArray)
                    val rect = Imgproc.boundingRect(areaPoints)

                    // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace
                    if (rect.area() > maxArea
                            && rect.x > BORDER_LEFT_X
                            && rect.x + rect.width < input.width() - BORDER_RIGHT_X
                            && rect.y > BORDER_TOP_Y
                            && rect.y + rect.height < input.height() - BORDER_BOTTOM_Y
                            || loopcounter - ploopcounter > 6) {
                        maxArea = rect.area()
                        maxRect = rect
                        ploopcounter++
                        loopcounter = ploopcounter
                        first = true
                    }
                    areaPoints.release()
                }
                contour.release()
            }

            mat.release()
            processed.release()
            if (contours.isEmpty()) {
                maxRect = Rect()
            }
            if (first && maxRect.area() > MIN_AREA) {
                if (DISPLAY) Imgproc.rectangle(output, maxRect, DISPLAY_COLOR, 2)
            }
            // Draw Borders
            if (DISPLAY) {
                // Display Data
                Imgproc.putText(output, "Area: " + getRectArea().toString() + " Midpoint: " + getRectMidpointXY().x.toString() + " , " +
                        getRectMidpointXY().y.toString() + " Selection: " + get(), Point(20.0, (input.height() - 20.0).toDouble()),
                        Imgproc.FONT_HERSHEY_PLAIN, 0.6, DISPLAY_COLOR, 1)
            }
            loopcounter++


        } catch (ignored: Exception) {

        }

        try {
            Thread.sleep(1000)
        } catch (ignored: InterruptedException) { }

        return output
    }

    private fun getRectHeight(): Int {
        return maxRect.height
    }

    private fun getRectWidth(): Int {
        return maxRect.width
    }

    private fun getRectX(): Int {
        return maxRect.x
    }

    private fun getRectY(): Int {
        return maxRect.y
    }

    fun getRectMidpointX(): Double {
        return getRectX() + getRectWidth() / 2.0
    }

    private fun getRectMidpointY(): Double {
        return getRectY() + getRectHeight() / 2.0
    }

    private fun getRectMidpointXY(): Point {
        return Point(getRectMidpointX(), getRectMidpointY())
    }

    fun getRectArea(): Double {
        return maxRect.area()
    }

    private var last = -1
    fun get(): Int {
        if (getRectArea() > MIN_AREA) {
            val p = getRectMidpointXY()
            if (Math.abs(p!!.x - LEFT.x) < VARIANCE && Math.abs(p.y - LEFT.y) < VARIANCE) last = 0
            if (Math.abs(p.x - CENTER.x) < VARIANCE && Math.abs(p.y - CENTER.y) < VARIANCE) last = 1
            if (Math.abs(p.x - RIGHT.x) < VARIANCE && Math.abs(p.y - RIGHT.y) < VARIANCE) last = 2
        }
        return last
    }


    companion object {
        var DISPLAY = true
        var DISPLAY_COLOR = Scalar(0.0, 0.0, 200.0)
        var LOWER_LIMIT = Scalar(100.0, 0.0, 0.0, 0.0)
        var UPPER_LIMIT = Scalar(255.0, 80.0, 80.0, 255.0)
        var BORDER_LEFT_X = 0 //amount of pixels from the left side of the cam to skip

        var BORDER_RIGHT_X = 0 //amount of pixels from the right of the cam to skip

        var BORDER_TOP_Y = 120 //amount of pixels from the top of the cam to skip

        var BORDER_BOTTOM_Y = 0 //amount of pixels from the bottom of the cam to skip


        //y is fot the outpiut
        var LEFT = Point(50.d, 120.d)
        var CENTER = Point(160.d, 120.d)
        var RIGHT = Point(270.d, 120.d)

        var VARIANCE = 50
        var MIN_AREA = 2000.0
    }
}
