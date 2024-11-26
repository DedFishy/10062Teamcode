package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionMechanismSubsystem;
import org.firstinspires.ftc.teamcode.utils.WaitTimer;

public class AutonRetract extends CommandBase {
    private final ExtensionMechanismSubsystem extension;
    private double extensionTime;
    private WaitTimer waitTimer;


    public AutonRetract(ExtensionMechanismSubsystem extension, double extensionTime) {

        this.extension = extension;
        this.extensionTime = extensionTime;
        this.waitTimer = new WaitTimer(extensionTime);

        addRequirements(extension);
    }

    @Override
    public void initialize() {
        waitTimer.startTimer();
    }

    @Override
    public void execute() {
        extension.setExtensionPower(-0.4);
    }

    @Override
    public boolean isFinished() {
        return waitTimer.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        extension.setExtensionPower(0);
    }
}
