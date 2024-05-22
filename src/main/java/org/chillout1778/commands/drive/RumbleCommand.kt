package org.chillout1778.commands.drive

import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls

// Just for fun
class RumbleCommand(private val seconds: Double = 1.0, private val amount: Double = 1.0): Command() {
    private val timer = Timer()

    override fun initialize() {
        timer.reset()
        timer.start()
        Controls.setRumble(amount)
    }

    override fun isFinished() = timer.get() > seconds

    override fun end(interrupted: Boolean) {
        Controls.operator.hid.setRumble(RumbleType.kBothRumble, 0.0)
        timer.stop() // unnecessary but seems polite
    }
}
