package org.chillout1778.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.lib.Util

object IntakeWrist: SubsystemBase(), Sendable {
    enum class State(val angle: Double?) {
        Up(Constants.Intake.upAngle),
        Down(Constants.Intake.downAngle),
        Undefined(null)
    }

    var currentState: State = State.Undefined
    private val motor = Util.neo(Constants.Ids.INTAKE_WRIST)
    private val encoder: SparkAbsoluteEncoder = IntakeRollers.motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
    private val positionPID = Constants.Intake.makeWristPID()
    var setpoint: Double = position

    init {
        motor.idleMode = CANSparkBase.IdleMode.kBrake
        Shuffleboard.getTab("Intake").add("Intake", this).withSize(3,6)
    }

    val position: Double get() {
            var raw = encoder.position
            if (raw < 0.15)
                raw = 1.0
            val fromHorizontal = 0.8 - raw
            return (fromHorizontal / 2.0) * 2.0*Math.PI
        }
    private var feedback = 0.0

    override fun periodic() {
        feedback = positionPID.calculate(position, setpoint)
        motor.setVoltage(feedback)
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.clearProperties()
        builder.addDoubleProperty("pos (deg)", { Math.toDegrees(position) }, {})
    }

}
