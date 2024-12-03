package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GrabberMechanismSubsystem;
import org.firstinspires.ftc.teamcode.utils.WaitTimer;

public class AutonGrabberRelease extends CommandBase {

    private final GrabberMechanismSubsystem grabber;

    public AutonGrabberRelease(GrabberMechanismSubsystem grabber) {

        this.grabber = grabber;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        grabber.setGrabberPower(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        grabber.setGrabberPower(0);
    }
}
