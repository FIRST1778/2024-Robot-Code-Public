package org.chillout1778.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.signals.InvertedValue
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Robot
import org.chillout1778.commands.drive.DriveCommand


// Swerve math resources:
// https://dominik.win/blog/programming-swerve-drive/
// https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf

object Swerve: SubsystemBase(), Sendable {
    private val gyro = Pigeon2(Constants.Ids.PIGEON)

    fun initGyro() {
        try {
            if (Robot.redAlliance()) {
                gyro.setYaw(180.0)
            } else {
                gyro.setYaw(0.0)
            }
        }catch(e: Exception){
            gyro.setYaw(0.0)
        }
    }

    init {
        defaultCommand = DriveCommand()
        Shuffleboard.getTab("Swerve").add("Swerve", this).withSize(2, 4)
    }

    private fun moduleTranslation(x: Double, y: Double) = 
        Translation2d(x,y) * Constants.Swerve.moduleXY

    private val modules = arrayOf(
        SwerveModule(
            name = "front left",
            encoderOffset = Math.toRadians(136.3), // *****
            driveMotorId = 1,
            turnMotorId = 5,
            turnCanCoderId = 9,
            driveInversion = InvertedValue.CounterClockwise_Positive,
            translation = moduleTranslation(1.0, 1.0)
        ),
        SwerveModule(
            name = "front right",
            encoderOffset = Math.toRadians(105.82), // *****
            driveMotorId = 2,
            turnMotorId = 6,
            turnCanCoderId = 10,
            driveInversion = InvertedValue.Clockwise_Positive,
            translation = moduleTranslation(1.0, -1.0)
        ),
        SwerveModule(
            name = "back right",
            encoderOffset = Math.toRadians(90.26), // *****
            driveMotorId = 3,
            turnMotorId = 7,
            turnCanCoderId = 11,
            driveInversion = InvertedValue.Clockwise_Positive,
            translation = moduleTranslation(-1.0, -1.0)
        ),
        SwerveModule(
            name = "back left",
            encoderOffset = Math.toRadians(.79), // *****
            driveMotorId = 4,
            turnMotorId = 8,
            turnCanCoderId = 12,
            driveInversion = InvertedValue.CounterClockwise_Positive,
            translation = moduleTranslation(-1.0, 1.0)
        ),
    )

    private val kinematics = SwerveDriveKinematics(
        *modules.map{it.translation}.toTypedArray()
    )

    private val odometry = SwerveDriveOdometry(
        kinematics,
        gyro.rotation2d,
        modules.map{it.position}.toTypedArray(),
    )


    val estimatedPose: Pose2d get() = odometry.poseMeters

    private fun driveChassisSpeeds(speeds: ChassisSpeeds) {
        val states = kinematics.toSwerveModuleStates(speeds)

        // Reduce module speeds so that none are faster than the
        // maximum.  The ratio between speeds is kept the same.
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
            Constants.Swerve.maxSpeed)

        for (i in states.indices)
            modules[i].drive(states[i])

        odometry.update(
            gyro.rotation2d,
            modules.map{it.position}.toTypedArray()
        )
    }

    fun drive(x: Double, y: Double, rot: Double) {
        driveChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, estimatedPose.rotation)
        )
    }

    init {
        for (mod in modules)
            mod.maybeResetRelative()
    }

    override fun periodic() {
        for (mod in modules)
             mod.maybeResetRelative()
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!! 
        builder.clearProperties()
        builder.addDoubleProperty("odometry angle", {estimatedPose.rotation.degrees}, {} )
    }

    // TODO: still uses odometry, not pose estimation
    fun configureHolonomic() {
        AutoBuilder.configureHolonomic(
            { estimatedPose },
            { pose: Pose2d ->
                odometry.resetPosition(
                    gyro.rotation2d,
                    modules.map{it.position}.toTypedArray(),
                    pose
                )
            },
            { kinematics.toChassisSpeeds(*modules.map{it.state}.toTypedArray()) },
            { speeds: ChassisSpeeds -> driveChassisSpeeds(speeds) },
            HolonomicPathFollowerConfig(
                PIDConstants(3.0,0.0,0.0), //translation
                PIDConstants(3.0 ,0.0,0.0), //rotation (this could be slower...)
                4.5,
                Constants.Swerve.moduleRadius,
                ReplanningConfig()
            ),
            {
                Robot.redAlliance()
            },
            this,
        )
    }
}
