package org.chillout1778

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sqrt

object Constants {
    object Ids {
        // ID layout for all devices on the CAN bus.

        // These need only be unique for a single manufacturer and
        // device type.  So a CANcoder and a Kraken can presumably have
        // the same ID without conflicting.  Not saying that we should
        // do that...
        //   https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
        // The actual CAN ID must be between 0 and 31.

        // === Swerve layout ===
        // Positions:     FL  FR  BR  BL
        // Drive motors   1,  2,  3,  4
        // Turn motors    5,  6,  7,  8
        // Turn encoders  9,  10, 11, 12

        const val CLIMBER_RIGHT_MASTER = 13
        const val CLIMBER_RIGHT_SLAVE  = 14
        const val CLIMBER_LEFT_MASTER  = 15
        const val CLIMBER_LEFT_SLAVE   = 16

        const val INTAKE_ROLLERS = 17
        const val INTAKE_WRIST = 18

        const val SHOOTER_WRIST_MASTER = 19
        const val SHOOTER_WRIST_SLAVE = 20

        const val SHOOTER_ROLLERS = 21

        const val SHOOTER_BOTTOM_FLYWHEEL = 23
        const val SHOOTER_TOP_FLYWHEEL = 22

        const val PIGEON = 30
    }

    object Climber {
        const val CIRCUMFERENCE = 0.028*PI // caliper measured 28mm diameter
        const val REDUCTION = 12.0/40.0 * 18.0/56.0
    }


    object Intake {
        // up: 0.261 rev; down: 0/1 rev; horizontal: 0.818 rev

        val upAngle = Math.toRadians(90.0)
        val downAngle = Math.toRadians(-25.0)

        fun makeWristPID() = PIDController(3.0, 0.0, 0.0)
    }

    object Lights {
        // 7 each on the back, 10 each on the front
        const val LED_COUNT = 34
    }

    object Shooter {
        private var distanceValues: Array<Pair<Double, Double>> = arrayOf(
            // distance to angle (degrees)
            1.34 to 0.0,
            1.94 to 9.75,
            2.715 to 18.75,
            3.35 to 24.0,
            3.89 to 27.75,
            4.25 to 28.75,
            4.9 to 30.75,
            5.25 to 31.75,
            5.73 to 32.75,
            6.75 to 33.0
        )
        private var speedValues: Array<Pair<Double, Double>> = arrayOf(
            // distance to angle (degrees)
            1.34 to (2500.0 * 2*Math.PI / 60.0),
            3.0 to (5000.0 * 2*Math.PI / 60.0)

        )
        val angleAimMap = InterpolatingDoubleTreeMap().apply {
            for((distance, angle) in distanceValues){
                put(distance, Math.toRadians(angle))
            }
        }
        private val flywheelSpeedMap = InterpolatingDoubleTreeMap().apply {
            for((distance, speed) in speedValues){
                put(distance, speed)
            }
        }

        const val ANGLE_REDUCTION = (12.0/66.0) * (18.0/66.0) * (16.0/64.0)

        val shootingSillyOffset = Math.toRadians(3.0)

        val ampAngle = Math.toRadians(147.0)
        val storedAngle = Math.toRadians(0.25)
        val shuttleAngle = Math.toRadians(5.0)
        fun flywheelShootingSpeed(n: Double) : Double{ return flywheelSpeedMap.get(n)}
        const val CONTINUOUS_FLYWHEEL_SPEED: Double = 1000.0 * 2*Math.PI / 60.0
        const val SHOOTING_SPEED: Double = 5000.0 * 2.0*Math.PI / 60.0
        const val SHUTTLE_SHOOTING_SPEED: Double = 2750.0 * 2*Math.PI / 60.0


        private const val MAX_WRIST_VELOCITY = 12.5
        private const val MAX_WRIST_ACCELERATION = 30.0

        private val wristTrapezoidConstraints = TrapezoidProfile.Constraints(MAX_WRIST_VELOCITY, MAX_WRIST_ACCELERATION)
        fun makeWristPID() = ProfiledPIDController(17.0, 0.5, 0.0, wristTrapezoidConstraints)
        fun makeFlywheelPID() = PIDController(0.02, 0.0, 0.0)
    }

    object Swerve {
        val moduleXY = Units.inchesToMeters(10.365)
        val moduleRadius = sqrt(moduleXY.pow(2.0) * 2.0)

        const val DRIVE_REDUCTION = 1.0/ 5.35714285714
        const val ANGLE_REDUCTION = 7.0 / 150.0

        val wheelRadius = Units.inchesToMeters(2.0)
        private const val KRAKEN_FREE_SPEED = 5800.0 * PI / 30.0 // with FOC
        private val theoreticalMaxSpeed = KRAKEN_FREE_SPEED * DRIVE_REDUCTION * wheelRadius
        private val theoreticalMaxAngularSpeed = theoreticalMaxSpeed / moduleRadius

        val maxSpeed = theoreticalMaxSpeed
        val maxAngularSpeed = theoreticalMaxAngularSpeed

        fun makeModuleTurnPID() = PIDController(7.0, 0.0, 0.0)
        fun makeDrivetrainTurnPID() = PIDController(.4, 0.0, 0.1).apply { enableContinuousInput(-Math.PI, Math.PI) }
    }
}
