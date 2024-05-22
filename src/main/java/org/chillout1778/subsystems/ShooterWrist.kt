package org.chillout1778.subsystems

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.lib.Util
import kotlin.math.abs

object ShooterWrist: SubsystemBase(), Sendable {
    enum class State(val defaultAngle: Double?) { // angle MUST BE IN RADIANS
        Amp(Constants.Shooter.ampAngle),
        Stored(Constants.Shooter.storedAngle),
        Shuttle(Constants.Shooter.shuttleAngle),
        Test(Constants.Shooter.storedAngle),
        Eject(Math.toRadians(15.0)),
        Source(Math.toRadians(115.0)),
        Tracking(null)
        ;
        val angle: Double get() {
            return if (this == Test) {
                angleOffset
            } else {
                defaultAngle ?: position
            }
        }
    }

    var angleOffset: Double = Math.toRadians(1.0)

    private var anglingEnabled: Boolean = true
    var zeroed = false

    fun offsetUpCommand() = Commands.runOnce({
        angleOffset += Math.toRadians(0.25)
    }, this)!!
    fun offsetDownCommand() = Commands.runOnce({
        angleOffset -= Math.toRadians(0.25)
    }, this)!!

    var currentState = State.Stored
    var requestedState = State.Stored

    val motorMaster = Util.neo(Constants.Ids.SHOOTER_WRIST_MASTER).apply {
        burnFlash()
    }
    val motorSlave = Util.neo(Constants.Ids.SHOOTER_WRIST_SLAVE).apply {
        follow(motorMaster, true)
        burnFlash()
    }

    private val relativeEncoder = motorMaster.encoder!!

    private val positionPID = Constants.Shooter.makeWristPID()

    init {
        Constants.Shooter //Does weird things if the profiledPID isn't created
        Shuffleboard.getTab("Shooter").add("Shooter Wrist", this).withSize(3,6)
    }

    var setpoint: Double = position
        set(n) {field = wrapAngle(n)}

    private val positionTolerance = Math.toRadians(2.0)
    private val lazyPositionTolerance = Math.toRadians(5.0)


    val atSetpoint: Boolean
        get() = abs(setpoint - position) < (if(requestedState != State.Amp && requestedState != State.Source) positionTolerance else lazyPositionTolerance)

    val position: Double get(){
        return relativeEncoder.position * Constants.Shooter.ANGLE_REDUCTION * 2.0*Math.PI
    }

    private fun wrapAngle(angle: Double): Double{
        return when{
            angle > Constants.Shooter.ampAngle -> Constants.Shooter.ampAngle
            angle < Constants.Shooter.storedAngle -> Constants.Shooter.storedAngle
            else -> angle
        }
    }

    val optimalAngle: Double
        get() = (Constants.Shooter.angleAimMap.get(Vision.speakerTagDistance) + angleOffset)
            .coerceIn(0.0, Math.toRadians(40.0))


    override fun periodic() {
        val feedforward = 0.0
        val feedback = positionPID.calculate(position, setpoint)
        if(anglingEnabled && zeroed) {
            motorMaster.setVoltage(feedback + feedforward)
        }
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.clearProperties()
        builder.addDoubleProperty("pos (deg)", { Math.toDegrees(position) }, {})
        builder.addDoubleProperty("setpoint (deg)", {Math.toDegrees((setpoint))}, {})
        builder.addDoubleProperty("angle offset", {Math.toDegrees(angleOffset)}, {})
    }
}
