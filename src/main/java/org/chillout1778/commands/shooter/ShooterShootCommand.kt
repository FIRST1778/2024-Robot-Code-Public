package org.chillout1778.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Controls
import org.chillout1778.commands.lights.BlinkCommand
import org.chillout1778.subsystems.ShooterRollers
import org.chillout1778.subsystems.ShooterWrist
import org.chillout1778.subsystems.Vision

class ShooterShootCommand(val sub : Boolean = false) : Command() {
    init{
        addRequirements(ShooterRollers)
    }
    private var readyToShoot = false

    private val timer = Timer()

    private var blinked = false

    private fun alignedToShoot() : Boolean{
        return (ShooterWrist.atSetpoint &&
                ShooterWrist.currentState == ShooterWrist.State.Tracking &&
                Vision.tagPresent &&
                Vision.angledTowardsSpeaker)
    }

    override fun initialize() {
        blinked = false
        if (Controls.aligningToShuttle)
            ShooterRollers.revForShuttle()
        else
            ShooterRollers.revFlywheels()
        timer.stop()
        timer.reset()
    }

    override fun execute() {
        if (ShooterRollers.flywheelsAtSpeed && !blinked) {
            BlinkCommand(0,255,0).schedule()
            blinked = true
        }
        if (!readyToShoot && ShooterRollers.flywheelsAtSpeed) {
            if(Controls.driverApproval && (sub || (Controls.aligningToShuttle && (ShooterWrist.currentState == ShooterWrist.State.Shuttle)) || ShooterWrist.currentState == ShooterWrist.State.Test)) { //Doesn't care about timer
                readyToShoot = true
                ShooterRollers.suck()
            }
            else if (Controls.driverApproval && alignedToShoot()) {
                readyToShoot = true
                timer.start()
            }
        } else if(!ShooterWrist.atSetpoint &&
            ShooterWrist.currentState == ShooterWrist.State.Tracking){
            readyToShoot = false //Resets if angle is later found to be wrong
            timer.stop()
            timer.reset()
        }
        if(timer.get() > 0.15){
            ShooterRollers.suck()
        }
    }

    override fun isFinished(): Boolean {
        return !ShooterRollers.bottomLineBreak && !ShooterRollers.topLineBreak
    }

    override fun end(interrupted: Boolean) {
        readyToShoot = false
        ShooterRollers.stopRollers()
        ShooterRollers.stopFlywheels()
    }
}
