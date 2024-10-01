package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * The command to drive along the y axis
 */
public class AutonDriveYDistanceCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem drive;

    //Initializes some important values
    private final double driveDistance;
    private final double tolerance = 0.5;
    private double newPosition;
    private double yDrive;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonDriveYDistanceCommand(DriveSubsystem subsystem, double driveDistance, double yDrive) {
        drive = subsystem;
        this.driveDistance = driveDistance;

        //Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        this.newPosition = driveDistance + drive.getPose().getY();
    }


    @Override
    public void execute() {
        if (newPosition >= drive.getPose().getY()) {
            drive.drive(0,yDrive,0,0.5);
        }
        else {
            drive.drive(0,-yDrive,0,0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        double distanceToNewPose = Math.abs(drive.getPose().getY() - newPosition);
        return distanceToNewPose <= tolerance;
    }
}