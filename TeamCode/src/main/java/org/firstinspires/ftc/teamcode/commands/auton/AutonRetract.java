package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionMechanismSubsystem;
import org.firstinspires.ftc.teamcode.utils.WaitTimer;

public class AutonRetract extends CommandBase {
    private final ExtensionMechanismSubsystem extension;


    public AutonRetract(ExtensionMechanismSubsystem extension) {

        this.extension = extension;
        addRequirements(extension);
    }


    @Override
    public void execute() {
        extension.setExtensionPower(-0.4);
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
