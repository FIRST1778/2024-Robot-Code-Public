package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterWrist

class ShooterTrackCommand: Command() {
    init {
        addRequirements(ShooterWrist)
    }

    override fun initialize() {
        ShooterWrist.requestedState = ShooterWrist.State.Tracking
        ShooterWrist.setpoint = ShooterWrist.optimalAngle
    }

    override fun execute() {
        ShooterWrist.setpoint = ShooterWrist.optimalAngle
        if(ShooterWrist.atSetpoint) ShooterWrist.currentState = ShooterWrist.State.Tracking
    }

    override fun isFinished(): Boolean {
        return false
    }
}
