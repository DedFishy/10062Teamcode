package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GrabberMechanismSubsystem;
import org.firstinspires.ftc.teamcode.utils.WaitTimer;

public class AutonGrabberRelease extends CommandBase {

    private final GrabberMechanismSubsystem grabber;
    private final WaitTimer timer;

    public AutonGrabberRelease(GrabberMechanismSubsystem grabber) {

        this.grabber = grabber;
        this.timer = new WaitTimer(5);
    }

    @Override
    public void initialize() {
        timer.startTimer();
    }

    @Override
    public void execute() {
        grabber.setGrabberPower(1);
    }

    @Override
    public boolean isFinished() {
        return timer.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.setGrabberPower(0);
    }
}
