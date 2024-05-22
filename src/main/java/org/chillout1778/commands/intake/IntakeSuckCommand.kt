package org.chillout1778.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.IntakeRollers

class IntakeSuckCommand(private val lazy : Boolean = false): Command() {
    init {
        addRequirements(IntakeRollers)
    }

    override fun execute() {
        if(!lazy) {
            IntakeRollers.suck()
        }else{
            IntakeRollers.lazySuck()
        }
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        IntakeRollers.off()
    }
}
