package org.chillout1778.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkMax
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.chillout1778.Constants
import org.chillout1778.lib.Util
import kotlin.math.abs

class ClimberHook(masterId: Int, slaveId: Int, private val inversion : Double): Sendable {
    private val master: CANSparkMax = Util.neo(masterId).apply {
        idleMode = CANSparkBase.IdleMode.kBrake
        setSmartCurrentLimit(50)
        burnFlash()
    }
    private val slave: CANSparkMax = Util.neo(slaveId).apply {
        follow(master, false) // KEEP FALSE
        idleMode = CANSparkBase.IdleMode.kBrake
        setSmartCurrentLimit(50)
        burnFlash()
    }

    init {
        Shuffleboard.getTab("Climbers")
            .add("Hook ($masterId $slaveId)", this)
    }

    val position get() =
        master.encoder.position * Constants.Climber.CIRCUMFERENCE * Constants.Climber.REDUCTION * inversion
    private val velocity get() =
        master.encoder.velocity * Constants.Climber.CIRCUMFERENCE * Constants.Climber.REDUCTION * inversion / 60.0
    val stopped: Boolean get() = abs(velocity) < 0.1 //500.0

    fun retract() {
        master.setVoltage(11.0 * inversion)
    }
    fun slowRetract(){
        master.setVoltage(2.0 * inversion)
    }
    fun stop() {
        master.setVoltage(0.0)
    }
    fun zero() {
        master.encoder.position = 0.0
    }
    fun extend() {
        master.setVoltage(-12.0 * inversion)
    }


    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.clearProperties()
        builder.addDoubleProperty("position", {position}, {})
    }
}
