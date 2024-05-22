package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterRollers

class CenterNoteCommand : Command(){
    init{
        addRequirements(ShooterRollers)
    }

    private val timer = Timer()

    private var passed = false

    private var noteAtStart = false

    override fun initialize() {
        noteAtStart = ShooterRollers.noteStored
        passed = false
        timer.reset()
        timer.start()
        ShooterRollers.reverseFlywheels()
    }

    override fun execute() {
        if(timer.get() < 0.1) {
            ShooterRollers.mediumSuck()
        }else if (ShooterRollers.topLineBreak && !passed){
            ShooterRollers.slowSpit()
        }else if(!passed) {
            ShooterRollers.stopRollers()
            passed = true
        }
    }

    override fun isFinished(): Boolean {
        return passed || !noteAtStart
    }

    override fun end(interrupted: Boolean) {
        ShooterRollers.stopFlywheels()
        ShooterRollers.stopRollers()
    }
}