package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose
import kotlin.math.absoluteValue

/**
 * this is like my .vimrc but for kotlin lol
 */
object Extensions {
    val Boolean.i get() = if (this) 1 else 0
    val Boolean.d get() = this.i.toDouble()
    val Number.d get() = this.toDouble()
    val Number.i get() = this.toInt()
    val Number.l get() = this.toLong()
    val Number.f get() = this.toFloat()

    val Double.mmToIn get() = this / 25.4
    val Double.cmToIn get() = this / 2.54
    val Double.inToMM get() = this * 25.4
    val Double.inToCM get() = this * 2.54

    val List<Double>.listAbs: List<Double> get() = this.map { it.absoluteValue }
    fun <E> ArrayList<E>.deepCopy(): ArrayList<E> {
        val copy = ArrayList<E>()
        this.forEach { copy.add(it) }
        return copy
    }
    val ArrayList<Double>.average: Double get() = this.sum() / this.size

    val Pose2d.pose: Pose get() = Pose(Point(x, y), Angle(heading, AngleUnit.RAD))
}
