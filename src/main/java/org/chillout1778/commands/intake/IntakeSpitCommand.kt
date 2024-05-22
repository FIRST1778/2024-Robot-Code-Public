package org.chillout1778.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.IntakeRollers

class IntakeSpitCommand: Command() {
    init {
        addRequirements(IntakeRollers)
    }

    override fun initialize() {
        IntakeRollers.spit()
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        IntakeRollers.off()
    }
}