package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * The command to drive along the y axis
 * Rotation Auton Command {@link org.firstinspires.ftc.teamcode.commands.auton.AutonDriveRotDistanceCommand}
 * Wait command {@link org.firstinspires.ftc.teamcode.commands.auton.WaitCommand}
 *
 * @author evokerking1, Mentors (DedFishy, Jetvac2)
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
     * @param driveDistance The distance to drive
     * @param yDrive The speed to drive
     */
    public AutonDriveYDistanceCommand(DriveSubsystem subsystem, double driveDistance, double yDrive) {
        drive = subsystem;
        this.driveDistance = driveDistance;
        this.yDrive = yDrive;

        //Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        drive.clearEncoderPulse();
        this.newPosition = driveDistance + drive.getDistance();
    }


    @Override
    public void execute() {
        if (newPosition >= drive.getDistance()) {
            drive.drive(0,yDrive,0,0.5,
                    Configuration.AutonFieldRelative);
        }
        else {
            drive.drive(0,-yDrive,0,0.5,
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