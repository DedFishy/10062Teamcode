package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class OffsetImu extends CommandBase {
    double offset;
    private final DriveSubsystem drive;
    private boolean performedOffset = false;

    public OffsetImu(DriveSubsystem driveSubsystem, double offset) {
        this.offset = offset;
        this.drive = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        drive.setImuOffset(offset);
        performedOffset = true;
    }

    @Override
    public boolean isFinished() {
        return performedOffset;
    }
}
