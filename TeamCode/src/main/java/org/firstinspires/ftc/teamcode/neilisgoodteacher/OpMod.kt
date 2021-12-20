package org.firstinspires.ftc.teamcode.neilisgoodteacher

import com.qualcomm.robotcore.eventloop.opmode.OpMode

class OpMod : OpMode() {
    override fun init() {
        val student1 = Student(50)
        val student2 = Student(100)
        student1.health
        student2.health
        student1.die()
        // student1.health = 50
        // student2.health = 100
        val a = Neil(1000000)

    }

    override fun loop() {

    }
}