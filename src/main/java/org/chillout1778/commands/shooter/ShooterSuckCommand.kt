package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterRollers

class ShooterSuckCommand(private val lazy: Boolean = false): Command() {
    init{
        addRequirements(ShooterRollers)
    }

    override fun initialize() {
        if(!ShooterRollers.noteStored) {
            if (lazy) ShooterRollers.lazySuck()
            else ShooterRollers.suck()
        }
    }

    override fun execute() {
        if(ShooterRollers.bottomLineBreak && !lazy){
            ShooterRollers.mediumSuck()
        }
    }
    override fun isFinished(): Boolean {
        return ShooterRollers.topLineBreak || (lazy && ShooterRollers.bottomLineBreak)
    }

    override fun end(interrupted: Boolean) {
        ShooterRollers.stopRollers()
    }
}