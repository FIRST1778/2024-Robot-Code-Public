package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterRollers

class RevFlywheelsCommand(private val shuttle: Boolean = false, private val time: Double = 0.0) : Command(){

    init{
        addRequirements(ShooterRollers)
    }

    private val timer = Timer()

    override fun initialize() {
        if(!shuttle) {
            ShooterRollers.revFlywheels()
        }else{
            ShooterRollers.revForShuttle()
        }
        timer.reset()
        timer.start()
    }

    override fun isFinished(): Boolean {
        return timer.get() > time || time == 0.0
    }

    override fun end(interrupted : Boolean){
        if(!interrupted && time != 0.0) {
            ShooterRollers.stopFlywheels()
        }
    }
}