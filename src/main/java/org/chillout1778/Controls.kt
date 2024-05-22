package org.chillout1778

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.chillout1778.commands.drive.RumbleCommand
import org.chillout1778.commands.climber.ClimberExtendCommand
import org.chillout1778.commands.climber.ClimberLowerCommand
import org.chillout1778.commands.climber.ClimberZeroCommand
import org.chillout1778.commands.intake.*
import org.chillout1778.commands.shooter.*
import org.chillout1778.subsystems.*

object Controls {
    private val driver = CommandJoystick(0)
    val operator = CommandXboxController(1)

    private const val FULL_OPERATOR_CONTROL = false

    val aligningToShoot get() = operator.hid.leftTriggerAxis > 0.5
    //driver.getRawAxis(7) > 0.75

    val driverApproval get() = driver.getRawAxis(7) > 0.9 || aligningToShoot || operator.hid.getRawButton(9) || Robot.isAutonomous
    val aligningToSource get() = if(Robot.isAutonomous) false else if(!FULL_OPERATOR_CONTROL) driver.getRawAxis(5) > 0.9 else operator.hid.rightBumper  //Not sure if this axis ID is correct
    val aligningToAmp get() = operator.hid.leftBumper
    val aligningToShuttle get() = operator.hid.aButton
    val aligningToChain get() = driver.getRawAxis(4) > 0.9
    val aligningToSourceIntake get() = operator.hid.xButton

    val wantVision get() = true

    val driveX get() = if(Robot.isAutonomous)0.0 else if(!FULL_OPERATOR_CONTROL) driver.getRawAxis(2) else -operator.getRawAxis(1)
    val driveY get() = if(Robot.isAutonomous)0.0 else if(!FULL_OPERATOR_CONTROL) -driver.getRawAxis(3) else -operator.getRawAxis(0)
    val driveZ get() = if(Robot.isAutonomous)0.0 else if(!FULL_OPERATOR_CONTROL) driver.getRawAxis(0) else operator.getRawAxis(4)

    private val rumbleTrigger: Trigger = Trigger { Timer.getMatchTime().toInt() == 20 && !Robot.isAutonomous}
    fun setRumble(amount : Double){
        operator.hid.setRumble(GenericHID.RumbleType.kBothRumble, amount)
    }
    init {
        rumbleTrigger.onTrue(RumbleCommand())
        operator.rightTrigger()
            .onTrue(IntakeAngleCommand(IntakeWrist.State.Down).andThen(
            ParallelDeadlineGroup(
                ShooterSuckCommand(),
                IntakeSuckCommand()
            )).andThen(ParallelCommandGroup(IntakeAngleCommand(IntakeWrist.State.Up), CenterNoteCommand())))
            .onFalse(
                ParallelCommandGroup(
                    IntakeAngleCommand(IntakeWrist.State.Up),
                    IntakeStopCommand()
                )
            )

        operator.leftBumper()
            .onTrue(ShooterAngleCommand(ShooterWrist.State.Amp))
            .whileTrue(ShooterAmpShootCommand())
            .onFalse(ShooterAngleCommand(ShooterWrist.State.Stored))

        operator.a()
            .onTrue(ShooterAngleCommand(ShooterWrist.State.Shuttle))
            .whileTrue(ShooterShootCommand())
            .onFalse(ShooterAngleCommand(ShooterWrist.State.Stored))

        operator.leftTrigger()
            .onTrue(ShooterTrackCommand())
            .whileTrue(ShooterShootCommand())
//            .onTrue(ShooterAngleCommand(ShooterWrist.State.Test))
            .onFalse(ShooterAngleCommand(ShooterWrist.State.Stored))

        // TODO: if Limelight disconnects, buzz the controller and don't try to angle the shooter.

        operator.rightStick()
            .whileTrue(ShooterShootCommand(true))

        operator.x()
            .whileTrue(ShooterAngleCommand(ShooterWrist.State.Source).andThen(ShooterSuckCommand(true)).andThen(CenterNoteCommand()))
            .onFalse(ShooterAngleCommand(ShooterWrist.State.Stored).andThen(CenterNoteCommand()))

        operator.y()
            .whileTrue(ParallelCommandGroup(
                IntakeSpitCommand(),
                ShooterSpitCommand()
            ))

        operator.pov(0)
            .whileTrue(ClimberExtendCommand())

        operator.b()
            .whileTrue(IntakeSpitCommand())


        operator.back()
            .onTrue(ClimberZeroCommand())

        operator.pov(180)
            .whileTrue(ClimberLowerCommand())

        operator.pov(90)
            .onTrue(ShooterWrist.offsetUpCommand())
        operator.pov(270)
            .onTrue(ShooterWrist.offsetDownCommand())

        operator.start()
            .whileTrue(
                ShooterAngleCommand(ShooterWrist.State.Eject).andThen(
                    ParallelCommandGroup(
                        ShooterSuckCommand(),
                        IntakeSpitCommand()
                    )
                )
            )
            .onFalse(
                ShooterAngleCommand(ShooterWrist.State.Stored)
            )
    }
}
