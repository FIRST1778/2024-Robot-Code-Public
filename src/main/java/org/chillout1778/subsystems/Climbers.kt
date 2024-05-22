package org.chillout1778.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants

//4 NEOs, two locked together per arm=2 separate NEOs
object Climbers: SubsystemBase() {
    var extended: Boolean = false

    private val left = ClimberHook(Constants.Ids.CLIMBER_LEFT_MASTER, Constants.Ids.CLIMBER_LEFT_SLAVE, 1.0)
    private val right = ClimberHook(Constants.Ids.CLIMBER_RIGHT_MASTER, Constants.Ids.CLIMBER_RIGHT_SLAVE, -1.0)
    val hooks = arrayOf(left, right)
}
