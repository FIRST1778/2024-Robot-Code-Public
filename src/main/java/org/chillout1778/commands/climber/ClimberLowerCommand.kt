package org.chillout1778.commands.climber

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Climbers
import kotlin.math.abs

class ClimberLowerCommand: Command() {
    //tells climber to lower at a constant pace until it reaches zero, this is different from the zero command because it only retracts it, and will ideally end when interrupted
    init {
        addRequirements(Climbers)
    }

    override fun initialize() {
        for(hook in Climbers.hooks){
            hook.retract()
        }
    }

    var allRetracted = false
    var complete = false

    override fun execute() {
        allRetracted = true
        for(hook in Climbers.hooks){
            if(abs(hook.position) < 0.02){
                hook.stop()
            }else{
                allRetracted = false
            }
        }
        complete = allRetracted
    }

    override fun isFinished(): Boolean {
        return complete
    }

    override fun end(interrupted: Boolean) {
        for(hook in Climbers.hooks){
            hook.stop()
        }
        if(!interrupted){
            Climbers.extended = false
        }
    }
}