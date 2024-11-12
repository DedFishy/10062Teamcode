package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * The command to drive along the x axis
 */
@Deprecated
public class AutonDriveXDistanceCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem drive;

    //Initializes some important values
    private final double driveDistance;
    private final double tolerance = 0.1;
    private double newPosition;
    private double xDrive;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonDriveXDistanceCommand(DriveSubsystem subsystem, double driveDistance, double xDrive) {
        drive = subsystem;
        this.driveDistance = driveDistance;

        //Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        this.newPosition = driveDistance + drive.getDistance();
    }


    @Override
    public void execute() {
        if (newPosition >= drive.getDistance()) {
            drive.drive(xDrive,0,0,0.5,
                    Configuration.AutonFieldRelative);
        }
        else {
            drive.drive(-xDrive,0,0,0.5,
                    Configuration.AutonFieldRelative);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0,0,0,false);
    }


    @Override
    public boolean isFinished() {
        double distanceToNewPose = Math.abs(drive.getDistance() - newPosition);
        return distanceToNewPose <= tolerance;
    }
}