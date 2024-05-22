package org.chillout1778

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.chillout1778.commands.climber.ClimberZeroCommand
import org.chillout1778.commands.drive.AutoAlignCommand
import org.chillout1778.commands.intake.IntakeAngleCommand
import org.chillout1778.commands.intake.IntakeSpitCommand
import org.chillout1778.commands.intake.IntakeStopCommand
import org.chillout1778.commands.intake.IntakeSuckCommand
import org.chillout1778.commands.shooter.*
import org.chillout1778.subsystems.*

object Robot: TimedRobot() {
    private var autonomousCommand: Command? = null
    fun start() {
        RobotBase.startRobot{this}
    }

    fun redAlliance() =  DriverStation.getAlliance().get() == Alliance.Red

    private lateinit var autoChooser: SendableChooser<Command>
    private val limelightField = Field2d()
    override fun robotInit() {
        CommandScheduler.getInstance().registerSubsystem(IntakeWrist)
        CommandScheduler.getInstance().registerSubsystem(Climbers)
        CommandScheduler.getInstance().registerSubsystem(Swerve)
        CommandScheduler.getInstance().registerSubsystem(IntakeRollers)
        CommandScheduler.getInstance().registerSubsystem(ShooterWrist)
        CommandScheduler.getInstance().registerSubsystem(ShooterRollers)
        CommandScheduler.getInstance().registerSubsystem(Vision)
        CommandScheduler.getInstance().registerSubsystem(Lights)

        Controls

        Swerve.initGyro()
        configureNamedCommands() // MUST BE BEFORE Swerve.configureHolonomic()
        Swerve.configureHolonomic()

        autoChooser = AutoBuilder.buildAutoChooser()
        Shuffleboard.getTab("Autos").apply {
            add(autoChooser).withSize(2, 1)
            add(limelightField).withSize(3, 3)
        }
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        Swerve.initGyro()
        Swerve.configureHolonomic()
        IntakeWrist.setpoint = IntakeWrist.position
        ShooterWrist.setpoint = ShooterWrist.position
        doTrajectoryAuto()
    }

    override fun autonomousExit() {
        autonomousCommand?.cancel()
    }

    override fun teleopInit() {
        IntakeWrist.setpoint = IntakeWrist.position
        ShooterWrist.setpoint = ShooterWrist.position
        IntakeAngleCommand(IntakeWrist.State.Up).schedule()
        ParallelCommandGroup(
            ShooterZeroCommand()
        ).andThen(ShooterAngleCommand(ShooterWrist.State.Stored)).schedule()
    }

    private fun doTrajectoryAuto() {
        autonomousCommand = autoChooser.selected
        ParallelCommandGroup(
            ShooterZeroCommand()
        ).andThen(
            ParallelCommandGroup(autonomousCommand!!, ClimberZeroCommand())
        ).andThen(
            ShooterAngleCommand(ShooterWrist.State.Stored)
        ).schedule()
    }

    private fun configureNamedCommands(){
        NamedCommands.registerCommand("Shoot",
            ParallelDeadlineGroup(
                ShooterShootCommand(),
                AutoAlignCommand(),
                ShooterTrackCommand()
            )
        )
        NamedCommands.registerCommand("Sub Shoot",
            ShooterShootCommand(true)
        )
        NamedCommands.registerCommand("Lazy Intake",
            ParallelCommandGroup(
                ShooterAngleCommand(ShooterWrist.State.Stored),
                IntakeAngleCommand(IntakeWrist.State.Down)
            ).andThen(
                ParallelRaceGroup(
                    ShooterSuckCommand(true),
                    IntakeSuckCommand(true),
                    WaitCommand(2.5)
                )
            ))
        NamedCommands.registerCommand("Intake",
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ShooterAngleCommand(ShooterWrist.State.Stored),
                    IntakeAngleCommand(IntakeWrist.State.Down)
                ),
                ParallelRaceGroup(
                    ShooterSuckCommand(),
                    IntakeSuckCommand(),
                    WaitCommand(2.0)
                ),
                ParallelCommandGroup(
                    IntakeAngleCommand(IntakeWrist.State.Up),
                    CenterNoteCommand(),
                    IntakeStopCommand()
                ),
                RevFlywheelsCommand()
            )
        )

        NamedCommands.registerCommand("Intake Spit",
            ParallelDeadlineGroup(
                WaitCommand(0.5),
                IntakeSpitCommand(),
                ShooterSpitCommand()
            )
        )
        NamedCommands.registerCommand("Intake Down",
            IntakeAngleCommand(IntakeWrist.State.Down))
        NamedCommands.registerCommand(
            "Lower Shooter",
            ShooterAngleCommand(ShooterWrist.State.Stored)
        )
        NamedCommands.registerCommand(
            "Rev",
            RevFlywheelsCommand()
        )
        NamedCommands.registerCommand(
            "Shooter Spit",
            RevFlywheelsCommand(false, 0.2)
        )
        NamedCommands.registerCommand(
            "Track",
            ShooterTrackCommand()
        )
    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }
}
