package org.chillout1778.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.IntakeRollers

class IntakeStopCommand: Command() {
    init {
        addRequirements(IntakeRollers)
    }

    override fun execute() {
        IntakeRollers.off()
    }

    override fun isFinished(): Boolean {
        return true
    }

    override fun end(interrupted: Boolean) {
        IntakeRollers.off()
    }
}
