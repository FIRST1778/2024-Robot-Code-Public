package org.chillout1778.commands.drive

import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.Controls.driveZ
import org.chillout1778.Robot
import org.chillout1778.lib.Util
import org.chillout1778.subsystems.Swerve
import org.chillout1778.subsystems.Vision
import kotlin.math.abs
import kotlin.math.sign

class DriveCommand: Command() {
    init {
        addRequirements(Swerve)
    }

    private val sourceAngle get() = Math.toRadians(if(Robot.redAlliance()) -120.0 else -60.0)
    private val shuttleAngle get() = Util.wrapAngle(Math.toRadians(if(Robot.redAlliance()) -146.68 else -33.3) + Constants.Shooter.shootingSillyOffset)

    private val ampAngle = Math.toRadians(-90.0)

    private fun square(n: Double) = n*n*sign(n)


    //Finds nearest change angle relative to the current chassis rotation
    private fun chainAngle(angleRad: Double): Double {
        val angle = Math.toDegrees(Util.wrapAngle(angleRad))
        return Util.wrapAngle(
            Math.toRadians(
                if(!Robot.redAlliance()){
                    if(angle < -120.0 || angle > 120.0) 180.0
                    else if(angle in 0.0..120.0) 60.0
                    else -60.0
                }else{
                    if(angle < 60.0 && angle > -60.0) 0.0
                    else if(angle in 60.0..180.0) 120.0
                    else -120.0
                }
            )
        )
    }

    private val turnPID = Constants.Swerve.makeDrivetrainTurnPID()

    override fun execute() {
        var x = Controls.driveX // x is forward (blue origin)
        var y = Controls.driveY // y is to the left (blue origin)
        if (Robot.redAlliance()) {
            x = -x
            y = -y
        }

        var usingAutoAlign: Boolean
        var autoAlignAngle = 0.0
        if (Util.deadband(driveZ) != 0.0) { // driver controller should always override
            usingAutoAlign = false
        } else {
            usingAutoAlign = true
            if (Controls.aligningToShoot) {
                autoAlignAngle = Vision.speakerTagAngle
            } else if (Controls.aligningToAmp) {
                autoAlignAngle = ampAngle
            } else if (Controls.aligningToShuttle) {
                autoAlignAngle = shuttleAngle
            } else if (Controls.aligningToSource) {
                autoAlignAngle = sourceAngle
            } else if(Controls.aligningToChain) {
                autoAlignAngle = chainAngle(Swerve.estimatedPose.rotation.radians)
            } else if(Controls.aligningToSourceIntake) {
                autoAlignAngle = Util.wrapAngle(sourceAngle + Math.PI)
            } else {
                usingAutoAlign = false
            }
        }
        val theta =
            if (usingAutoAlign)
                turnPID.calculate(
                    Swerve.estimatedPose.rotation.radians,
                    autoAlignAngle
                )
            else
                -square(Util.deadband(driveZ))

        val maxInput = if(Controls.aligningToShoot) 0.05 else 1.0 //allows minor shoot-on-the-move

        Vision.angledTowardsSpeaker = abs(theta) < Math.toRadians(1.0)

        if(!Robot.isAutonomous) {
            Swerve.drive(
                square(Util.deadband(x)).coerceIn(-maxInput, maxInput) * Constants.Swerve.maxSpeed,
                square(Util.deadband(y)).coerceIn(-maxInput, maxInput) * Constants.Swerve.maxSpeed,
                theta * Constants.Swerve.maxAngularSpeed
            )
        }
    }
}
