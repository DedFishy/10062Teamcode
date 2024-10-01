package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * The command to drive along the x axis
 */
public class AutonDriveXDistanceCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem drive;

    //Initializes some important values
    private final double driveDistance;
    private final double tolerance = 0.5;
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
        this.newPosition = driveDistance + drive.getPose().getX();
    }


    @Override
    public void execute() {
        if (newPosition >= drive.getPose().getX()) {
            drive.drive(xDrive,0,0,0.5);
        }
        else {
            drive.drive(-xDrive,0,0,0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        double distanceToNewPose = Math.abs(drive.getPose().getX() - newPosition);
        return distanceToNewPose <= tolerance;
    }
}