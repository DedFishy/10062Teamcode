package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionMechanismSubsystem;
import org.firstinspires.ftc.teamcode.utils.WaitTimer;

public class AutonExtend extends CommandBase {
    private final ExtensionMechanismSubsystem extension;


    public AutonExtend(ExtensionMechanismSubsystem extension) {

        this.extension = extension;


        addRequirements(extension);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        extension.setExtensionPower(0.8);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        extension.setExtensionPower(0);
    }
}
