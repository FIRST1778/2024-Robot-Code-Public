package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterRollers

class ShooterSpitCommand : Command() {
    init{
        addRequirements(ShooterRollers)
    }

    override fun initialize() {
        ShooterRollers.spit()
        ShooterRollers.reverseFlywheels()
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        ShooterRollers.stopRollers()
    }
}