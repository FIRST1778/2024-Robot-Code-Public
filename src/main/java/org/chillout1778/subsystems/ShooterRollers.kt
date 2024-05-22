package org.chillout1778.subsystems

import com.revrobotics.CANSparkBase
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.commands.shooter.ContinuousRollingCommand
import org.chillout1778.lib.Util
import kotlin.math.abs

object ShooterRollers: SubsystemBase(), Sendable {

    private class Flywheel(id: Int, val m: Double, val b: Double) {
        private val motor = Util.neo(id)
        private val pid = Constants.Shooter.makeFlywheelPID()
        val velocity get() = -motor.encoder.velocity * 2*Math.PI / 60.0
        var volts = 0.0
        fun periodic() {
            val feedforward = if (setpoint == 0.0) 0.0 else -(setpoint * m + b)
            val feedback = -pid.calculate(velocity, setpoint)
            volts = feedforward + feedback
            motor.setVoltage(volts)
        }
    }

    val flywheelsAtSpeed: Boolean
        get() = when{
            setpoint == 0.0 -> false // prevents immediately shooting
            Controls.aligningToShuttle -> abs(topFlywheel.velocity - setpoint) < 100.0
            topFlywheel.velocity < Constants.Shooter.flywheelShootingSpeed(Vision.speakerTagDistance) - 100.0 -> false
//            topFlywheel.velocity > setpoint + 100.0 -> false
            else -> true
        }
    private val rollers = Util.neo(Constants.Ids.SHOOTER_ROLLERS).apply {
        idleMode = CANSparkBase.IdleMode.kBrake
    }
    private val bottomFlywheel = Flywheel(Constants.Ids.SHOOTER_BOTTOM_FLYWHEEL, 0.0198, 0.319)
    private val topFlywheel = Flywheel(Constants.Ids.SHOOTER_TOP_FLYWHEEL, 0.0198, 0.351)

    private val rawTopLineBreak = DigitalInput(3)
    private val rawBottomLineBreak = DigitalInput(4)

    private var setpoint = 0.0 // rad/s

    var lineBreakOverride = false
    val noteStored: Boolean
        get() = !rawBottomLineBreak.get() && !lineBreakOverride

    val topLineBreak : Boolean
        get() = !rawTopLineBreak.get() && !lineBreakOverride

    val bottomLineBreak : Boolean
        get() = !rawBottomLineBreak.get() && !lineBreakOverride

    init {
        Shuffleboard.getTab("Shooter").add("ShooterRollers", this).withSize(3, 6)
        defaultCommand = ContinuousRollingCommand() // if we'd like
    }

    override fun periodic() {
        bottomFlywheel.periodic()
        topFlywheel.periodic()
    }

    fun suck() {
        rollers.set(-0.9)
    }

    fun mediumSuck(){
        rollers.set(-0.5)
    }

    fun lazySuck(){
        rollers.set(-.5)
    }

    fun spit(){
        rollers.set(0.6)
    }

    fun slowSpit() {
        rollers.set(0.15)
    }

    //prepares flywheels for shooting to speaker
    fun revFlywheels(){
        setpoint = Constants.Shooter.SHOOTING_SPEED
    }
    fun revForShuttle(){
        setpoint = Constants.Shooter.SHUTTLE_SHOOTING_SPEED
    }

    fun reverseFlywheels(){
        setpoint = -Constants.Shooter.CONTINUOUS_FLYWHEEL_SPEED
    }

    //shoots into amp
    fun shootAmp(){
        rollers.set(0.5)
    }

    fun continuousFlywheels() {
        setpoint = Constants.Shooter.CONTINUOUS_FLYWHEEL_SPEED
    }

    fun stopFlywheels(){
        setpoint = 0.0
    }

    fun stopRollers(){
        rollers.setVoltage(0.0)
    }


    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.clearProperties()
        builder.addBooleanProperty("Note Stored", { noteStored}, {})
        builder.addDoubleProperty("setpoint", { setpoint}, {})
        builder.addBooleanProperty("bottom line break", { bottomLineBreak}, {})
        builder.addBooleanProperty("top line break", { topLineBreak }, {})
        builder.addDoubleProperty("bottom velocity (rad/s)", { bottomFlywheel.velocity }, {})
        builder.addDoubleProperty("top velocity (rad/s)", { topFlywheel.velocity }, {})
    }
}
