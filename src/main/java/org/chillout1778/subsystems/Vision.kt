package org.chillout1778.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Subsystem
import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.Robot
import org.chillout1778.lib.LimelightHelpers
import org.chillout1778.lib.Util
import kotlin.math.atan
import kotlin.math.hypot

object Vision: Subsystem, Sendable {
    private const val NAME: String = "limelight"

    private val id7 = Pose3d(-8.308975, 1.442593,1.451102, Rotation3d(0.0, 0.0, 0.0))
    private val id4 = Pose3d(8.308467, 1.442593, 1.451102, Rotation3d(0.0, 0.0, Math.PI))

    private val speakerPose: Pose3d
        get() = if (Robot.redAlliance()) id4 else id7
    private val translationToSpeaker: Translation2d get() = Translation2d(speakerPose.x, speakerPose.y) - lastRobotPose.translation
    val speakerTagDistance : Double //METERS
        get() = hypot(translationToSpeaker.x, translationToSpeaker.y)
    val speakerTagAngle : Double // RADIANS, relative to gyro rotation, if robot is on the source side of the april tag this value is negative
        get() = Util.wrapAngle(
            -Constants.Shooter.shootingSillyOffset +
            (if (Robot.redAlliance()) Math.PI else 0.0) +
            atan(translationToSpeaker.y / translationToSpeaker.x)
        )

    private var lastRobotPose: Pose2d = Pose2d()

    val tagPresent: Boolean
        get() = LimelightHelpers.getTV(NAME)

    var angledTowardsSpeaker = false


    override fun periodic() {
        // Update raw robot pose if any tag is seen.
        if (tagPresent && Controls.wantVision) {
            lastRobotPose = LimelightHelpers.getBotPose2d(NAME)
        }
    }
    init {
        Shuffleboard.getTab("Limelight").add("Limelight", this).withSize(2, 2)
    }
    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.clearProperties()
        builder.addBooleanProperty("tagPresent", { tagPresent }, {})

    }
}
