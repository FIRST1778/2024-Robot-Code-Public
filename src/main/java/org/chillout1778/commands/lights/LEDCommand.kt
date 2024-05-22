package org.chillout1778.commands.lights

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Lights
import org.chillout1778.subsystems.ShooterRollers

class LEDCommand: Command() {
    init{
        addRequirements(Lights)
    }
    override fun runsWhenDisabled() = true // currently default, can change to true

    override fun execute() {
        // If we get an intake line break, that should set the LEDs to an
        // intermediate color, so we can start moving towards the speaker.

        if (ShooterRollers.bottomLineBreak || ShooterRollers.topLineBreak)
            Lights.write(255,255,255) // white
        else
            Lights.write(20,20,255) // blue
    }

    override fun end(interrupted: Boolean) {
        Lights.write(0,0,0)
    }
}
