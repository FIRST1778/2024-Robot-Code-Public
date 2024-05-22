package org.chillout1778.commands.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.subsystems.Swerve
import org.chillout1778.subsystems.Vision
import kotlin.math.abs

class AutoAlignCommand: Command() {
    init {
        addRequirements(Swerve)
    }

    private val turnPID = PIDController(.4, 0.0, 0.1).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun initialize() {
        Vision.angledTowardsSpeaker = false
    }

    override fun execute() {
        val theta =
            turnPID.calculate(
                Swerve.estimatedPose.rotation.radians,
                Vision.speakerTagAngle).coerceIn(-1.0, 1.0
            )

        Vision.angledTowardsSpeaker = abs(theta) < Math.toRadians(1.0)

        Swerve.drive(
            0.0,
            0.0,
            theta * Constants.Swerve.maxAngularSpeed
        )
    }
}
