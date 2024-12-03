package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystem;
import org.firstinspires.ftc.teamcode.utils.WaitTimer;

public class AutonPivotRaise extends CommandBase {

    private final PivotMechanismSubsystem pivot;
    private final double stationaryPower = 0.1;
    public AutonPivotRaise(PivotMechanismSubsystem pivot) {

        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPivotPower(0.8);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean finished) {
        pivot.setPivotPower(stationaryPower);
    }

}
