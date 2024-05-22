package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.subsystems.ShooterRollers
import org.chillout1778.subsystems.ShooterWrist

class ShooterAmpShootCommand: Command() {
    init{
        addRequirements(ShooterRollers)
    }

    private var shootingReady = false
    override fun execute() {
        if((Controls.driverApproval) && !shootingReady && ShooterWrist.currentState == ShooterWrist.State.Amp) {
            shootingReady = true
            ShooterRollers.shootAmp()
        }
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        shootingReady = false
        ShooterRollers.stopRollers()
    }
}