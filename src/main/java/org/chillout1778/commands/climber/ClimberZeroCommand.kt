package org.chillout1778.commands.climber

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.subsystems.Climbers

class ClimberZeroCommand: Command() {
    val timer = Timer()
    init {
        addRequirements(Climbers)
    }

    private var allStopped = false

    override fun initialize() {
        allStopped = false
        timer.reset()
        timer.start()
        for (hook in Climbers.hooks)
            hook.slowRetract()
    }

    override fun execute() {
        if (timer.get() < 0.25) {
            return
        }
        allStopped = true
        for (hook in Climbers.hooks) {
            if (hook.stopped) {
                hook.stop()
            } else {
                allStopped = false
            }
        }
        if (allStopped)
            cancel()
    }

    override fun isFinished(): Boolean {
        return allStopped
    }

    override fun end(interrupted: Boolean) {
        for (hook in Climbers.hooks) {
            hook.stop() // TODO: just in case?
            hook.zero()
        }
    }
}
