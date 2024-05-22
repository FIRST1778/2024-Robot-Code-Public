package org.chillout1778.subsystems

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.chillout1778.Constants
import org.chillout1778.lib.Util

class SwerveModule(
        val name: String,
        driveMotorId: Int, turnMotorId: Int, turnCanCoderId: Int,
        val encoderOffset: Double,
        val driveInversion: InvertedValue,
        val translation: Translation2d,
): Sendable {
    private val driveMotor = TalonFX(driveMotorId)
    private val turnMotor: CANSparkMax = Util.neo(turnMotorId)
    private val turnCanCoder = CANcoder(turnCanCoderId)
    private val turnPID: PIDController = Constants.Swerve.makeModuleTurnPID()

    // TODO: Voltage compensation???

    init {
        driveMotor.configurator.apply(
            MotorOutputConfigs().withInverted(driveInversion)
        )
        driveMotor.configurator.apply(
            FeedbackConfigs().withSensorToMechanismRatio(1.0)
                             .withRotorToSensorRatio(1.0)
        )
        driveMotor.configurator.apply(
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(45.0)
                .withSupplyCurrentThreshold(70.0)
                .withSupplyTimeThreshold(0.1)
                .withStatorCurrentLimit(80.0)
                .withStatorCurrentLimitEnable(true)
        )


//        driveMotor.configurator.apply(
//            AudioConfigs().withBeepOnBoot(false)
//        )
        driveMotor.setNeutralMode(NeutralModeValue.Brake)
        turnMotor.inverted = false
        turnMotor.idleMode = CANSparkBase.IdleMode.kCoast
        turnMotor.setSmartCurrentLimit(30)
        turnMotor.burnFlash()
        turnCanCoder.configurator.apply(
            MagnetSensorConfigs().withMagnetOffset(-encoderOffset / (2.0*Math.PI))
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        )
        turnPID.enableContinuousInput(-Math.PI, Math.PI) // TODO: make sure inputs are wrapped to this range
        resetRelative()
        Shuffleboard.getTab("Swerve").add(name, this).withSize(3,6)
    }

    private val driveVelocity get() = 2.0*Math.PI * driveMotor.velocity.value * Constants.Swerve.DRIVE_REDUCTION * Constants.Swerve.wheelRadius
    private val drivePosition get() = 2.0*Math.PI * driveMotor.position.value * Constants.Swerve.DRIVE_REDUCTION * Constants.Swerve.wheelRadius
    private val turnPosition get()  = -Util.wrapAngle(2.0*Math.PI * Constants.Swerve.ANGLE_REDUCTION * turnMotor.encoder.position)
    private val turnVelocity get()  = -Math.PI/30.0 * Constants.Swerve.ANGLE_REDUCTION * turnMotor.encoder.velocity

    val state get() = SwerveModuleState(
        driveVelocity, Rotation2d(turnPosition)
    )
    val position get() = SwerveModulePosition(
        drivePosition, Rotation2d(turnPosition)
    )

    private var turnStationaryTicks: Int = 0

    private fun resetRelative() {
        // Use the CANCoder's .getAbsolutePosition() API instead of
        // .getPosition().  Even though we tell the CANCoder to boot up
        // in absolute mode (see Util.kt), there were apparently some
        // firmware bugs that would rarely cause .getPosition() to report
        // incorrect values after startup.
        // https://store.ctr-electronics.com/blog/cancoder-firmware-update-22012/
        // https://discord.com/channels/887922855084425266/890436659450118254/1170883077195714590
        // https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/rev/NeoSteerControllerFactoryBuilder.java#L128C3-L128C3
        turnMotor.encoder.position = -turnCanCoder.absolutePosition.value / Constants.Swerve.ANGLE_REDUCTION
        turnStationaryTicks = 0
    }
    
    fun maybeResetRelative() {
        // Count how many ticks this turn motor has been stationary;
        // if we've waited long enough, we take this as a cue to reset
        // relative encoders back to absolute.
        if (turnVelocity < 0.3) {
            turnStationaryTicks += 1
        }
        if (turnStationaryTicks > 250) {
            resetRelative()
        }
    }

    private val driveMotorOut: VoltageOut = VoltageOut(0.0).withEnableFOC(true)

    fun drive(unoptimizedState: SwerveModuleState) {
        // Optimize the swerve module state (i.e., drive velocity
        // and turn position) so that we never turn more than 90 degrees.
        val rotation = Rotation2d(turnPosition)
        val optimizedState = SwerveModuleState.optimize(unoptimizedState, rotation)

        // Scale by cosine.  If the module is far from its
        // desired angle, then we drive it correspondingly less.
        val wantDriveVelocity = optimizedState.speedMetersPerSecond * (optimizedState.angle - rotation).cos
        val driveVoltage = wantDriveVelocity / Constants.Swerve.maxSpeed * 12.0

        val wantTurnPosition = Util.wrapAngle(optimizedState.angle.radians)
        val turnVoltage = -turnPID.calculate(turnPosition, wantTurnPosition)

        driveMotorOut.Output = driveVoltage
        driveMotor.setControl(driveMotorOut)
        turnMotor.setVoltage(Util.clamp(turnVoltage, 12.0))
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder!!
        builder.clearProperties()
        builder.addDoubleProperty("Turn position (deg)", {Math.toDegrees(turnPosition)}, {})
        builder.addDoubleProperty("Drive velocity", {driveVelocity}, {})
    }
}
