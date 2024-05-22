package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterRollers

class ContinuousRollingCommand: Command() {
    init {
        addRequirements(ShooterRollers)
    }

    override fun initialize() {
        if (!ShooterRollers.lineBreakOverride && ShooterRollers.noteStored) {
            ShooterRollers.continuousFlywheels()
        }
    }

    override fun end(interrupted: Boolean) {
        ShooterRollers.stopFlywheels()
    }
}