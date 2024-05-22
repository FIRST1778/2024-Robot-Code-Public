package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterWrist

class ShooterAngleCommand(private val shooterState: ShooterWrist.State): Command() {
    init {
        addRequirements(ShooterWrist)
    }

    override fun initialize() {
        ShooterWrist.requestedState = shooterState
        if (shooterState == ShooterWrist.State.Tracking) {
            DriverStation.reportError("tried to use ShooterAngleCommand with Tracking state", true)
            cancel()
        }
        if (shooterState == ShooterWrist.State.Test) {
            ShooterWrist.setpoint = ShooterWrist.angleOffset
        } else {
            ShooterWrist.setpoint = shooterState.defaultAngle ?: ShooterWrist.position
        }
    }
    override fun isFinished(): Boolean {
        return ShooterWrist.atSetpoint
    }

    override fun end(interrupted: Boolean) {
        if(!interrupted){
            ShooterWrist.currentState = shooterState
        }
    }
}
