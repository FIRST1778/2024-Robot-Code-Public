package org.chillout1778.commands.lights

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Lights

class BlinkCommand(private val r: Int, private val g: Int, private val b: Int, private val blinks: Int = 2): Command() {
    init{
        addRequirements(Lights)
    }

    private var count = 0
    private var ticks = 20 // per blink cycle

    override fun initialize() {
        count = blinks * ticks * 2
    }

    override fun execute() {
        if (count % (ticks*2) == 0)
            Lights.write(r,g,b)
        else if (count % (ticks*2) == ticks)
            Lights.write(0,0,0) // black
        ticks--
    }

    override fun isFinished() = ticks <= 0

    override fun end(interrupted: Boolean) {
        Lights.write(0,0,0)
    }
}
