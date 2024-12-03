package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystem;

public class AutonPivotLower extends CommandBase {

    private final PivotMechanismSubsystem pivot;

    public AutonPivotLower(PivotMechanismSubsystem pivot) {

        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPivotPower(-0.4);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean finished) {
        pivot.setPivotPower(0);
    }

}
