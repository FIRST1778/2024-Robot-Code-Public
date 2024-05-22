package org.chillout1778.subsystems

import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants
import org.chillout1778.lib.Util

object IntakeRollers: Subsystem {
    val motor = Util.neo(Constants.Ids.INTAKE_ROLLERS)

    fun suck() {
        motor.set(-.95)
    }

    fun lazySuck(){
        motor.set(-.4)
    }

    fun spit(){
        motor.set(.5)
    }

    fun  off() {
        motor.set(0.0)
    }
}
