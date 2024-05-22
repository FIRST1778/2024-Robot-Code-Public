package org.chillout1778.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Robot
import org.chillout1778.subsystems.IntakeWrist
import org.chillout1778.subsystems.ShooterRollers

class IntakeAngleCommand(private val inputState: IntakeWrist.State): Command() {
    init {
        addRequirements(IntakeWrist)
    }

    private var intakeState = inputState

    override fun initialize() {
        intakeState = if(ShooterRollers.noteStored && !Robot.isAutonomous){
            IntakeWrist.State.Up
        }else{
            inputState
        }
        IntakeWrist.setpoint = intakeState.angle ?: IntakeWrist.position
    }

    override fun isFinished() = true

    override fun end(interrupted: Boolean) {
        if (!interrupted){
            IntakeWrist.currentState = intakeState
        }
    }
}
