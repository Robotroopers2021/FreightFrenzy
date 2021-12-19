package org.firstinspires.ftc.teamcode.telemetryStuff

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import org.firstinspires.ftc.robotcore.external.Telemetry

object Dashboard {
    private var dashboardAdapter: TelemetryAdapter = TelemetryAdapter()
    private lateinit var telemetryImpl: Telemetry
    private var isUpdatingDashboard: Boolean = true

    fun init(telemImpl: Telemetry, shouldUpdate: Boolean) {
        telemetryImpl = telemImpl
        isUpdatingDashboard = shouldUpdate
    }

    fun fieldOverlay(): Canvas? {
        return if(isUpdatingDashboard) {
            dashboardAdapter.fieldOverlay()
        } else {
            null
        }
    }

    fun setHeader(v: String) {
        addSpace()
        addLine("------$v------")
    }

    fun addLine(v: String) {
        if(isUpdatingDashboard) {
            dashboardAdapter.addLine(v)
        }

        telemetryImpl.addLine(v)
    }

    fun addSpace() {
        addLine(" ")
    }

    fun update() {
        if (isUpdatingDashboard) {
            FtcDashboard.getInstance().sendTelemetryPacket(dashboardAdapter)
            dashboardAdapter = TelemetryAdapter()
        }
        telemetryImpl.update()
    }

    operator fun set(k: String, v: Any) {
        telemetryImpl.addData(k, v)

        if(isUpdatingDashboard) {
            dashboardAdapter.put(k, v)
        }
    }

}