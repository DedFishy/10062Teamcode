package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystem;

public class AutonPivotLower extends CommandBase {

    private final PivotMechanismSubsystem pivot;
    private double revolutions;

    public AutonPivotLower(PivotMechanismSubsystem pivot, double revolutions) {

        this.pivot = pivot;
        this.revolutions = revolutions;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPivotPower(-0.4);
    }


    @Override
    public boolean isFinished() {
        return pivot.getPivotPosition() <= revolutions;
    }

    @Override
    public void end(boolean finished) {
        pivot.setPivotPower(0);
    }

}
