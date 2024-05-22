package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.ShooterWrist
import org.chillout1778.subsystems.ShooterWrist.zeroed

class ShooterZeroCommand : Command() {
    init {
        addRequirements(ShooterWrist)
    }

    override fun initialize() {
        if(!zeroed) {
            ShooterWrist.motorMaster.encoder.position = 0.0
            zeroed = true
            ShooterWrist.setpoint = ShooterWrist.position
        }
    }

    override fun isFinished(): Boolean {
        return true
    }

    override fun end(interrupted: Boolean) {
        ShooterWrist.motorMaster.setVoltage(0.0)
    }
}