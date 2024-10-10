package org.firstinspires.ftc.teamcode.commands.auton;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DriveCircumference extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


    private final DriveSubsystem drive;
    private final double speed = 0.1;
    private final double tolerance = 0.01;
    private double encoderStartValue;

    /**
     * Makes the bot stop for a specified period
     */
    public DriveCircumference(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        this.encoderStartValue = drive.encoderRevolutions();
    }

    @Override
    public void execute() {
        if (1 >=encoderStartValue - drive.encoderRevolutions()) {
            drive.drive(0,speed,0,0.5);
        }
        else {
            drive.drive(0,-speed,0,0.5);
        }
    }


    @Override
    public boolean isFinished() {
        double distanceToNewPose = Math.abs(Math.abs(encoderStartValue - drive.encoderRevolutions()) - 1);
        return distanceToNewPose <= tolerance;
    }
}