package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * The command to drive along the y axis
 */
public class AutonDriveRotDistanceCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem drive;

    //Initializes some important values
    private final double rot;
    private final double tolerance = 0.5;
    private double newRotation;
    private double rotSpeed;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonDriveRotDistanceCommand(DriveSubsystem subsystem, double rot, double rotSpeed) {
        drive = subsystem;
        this.rotSpeed = rotSpeed;
        this.rot = rot;

        //Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        this.newRotation = rot + drive.getRotation();
    }


    @Override
    public void execute() {
        if (newRotation >= drive.getRotation()) {
            drive.drive(0,0,rotSpeed,0.5,
                    Configuration.AutonFieldRelative);
        }
        else {
            drive.drive(0,0,-rotSpeed,0.5,
                    Configuration.AutonFieldRelative);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        double distanceToNewPose = Math.abs(drive.getRotation()
                - newRotation);
        return distanceToNewPose <= tolerance;
    }
}