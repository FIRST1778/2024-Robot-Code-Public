package org.chillout1778.commands.climber

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.subsystems.Climbers
import org.chillout1778.subsystems.IntakeWrist
import kotlin.math.abs

class ClimberExtendCommand: Command() {
    init {
        addRequirements(Climbers, IntakeWrist)
    }

    override fun initialize() {
        for(hook in Climbers.hooks){
            hook.extend()
        }
        IntakeWrist.setpoint = IntakeWrist.State.Down.angle!!
    }

    var allExtended = false
    var complete = false

    override fun execute() {
        allExtended = true
        for(hook in Climbers.hooks){
            if(abs(hook.position) > 0.7){
                hook.stop()
            }else{
                allExtended = false
            }
        }
        complete = allExtended
    }

    override fun isFinished(): Boolean {
        return complete
    }

    override fun end(interrupted: Boolean) {
        for(hook in Climbers.hooks){
            hook.stop()
        }
        if(!interrupted){
            Climbers.extended = true
        }
    }
}
