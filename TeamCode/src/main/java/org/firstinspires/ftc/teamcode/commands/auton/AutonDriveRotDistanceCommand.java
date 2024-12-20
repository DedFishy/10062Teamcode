package org.firstinspires.ftc.teamcode.commands.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * The command to Rotate
 * Drive along Y axis Command {@link org.firstinspires.ftc.teamcode.commands.auton.AutonDriveYDistanceCommand}
 * Wait command {@link org.firstinspires.ftc.teamcode.commands.auton.WaitCommand}
 * @author evokerking1, Mentors (DedFishy, Jetvac2)
 */

public class AutonDriveRotDistanceCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem drive;

    //Initializes some important values
    private final double rot;
    private final double tolerance = 0.1;
    private double newRotation;
    private double rotSpeed;
    private final double rateOfDecay = 2;

    private final TelemetryPacket packet = new TelemetryPacket();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     * @param rot Distance to rotate in radians
     */
    public AutonDriveRotDistanceCommand(DriveSubsystem subsystem, double rot){
        drive = subsystem;
       
        this.rot = rot;

        //Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        drive.resetImu();
        this.newRotation = rot + drive.getRotation();
    }


    @Override
    public void execute() {
        rotSpeed = 0.7; //Math.signum(drive.getRotation() - newRotation) * Math.pow(drive.getRotation()
                //- newRotation, rateOfDecay) / Math.pow (180, rateOfDecay);

        if (newRotation >= drive.getRotation()) {
            drive.drive(0,0,rotSpeed,0.5,
                    Configuration.AutonFieldRelative);
        }
        else {
            drive.drive(0,0,-rotSpeed,0.5,
                    Configuration.AutonFieldRelative);
        }

        packet.put("Rotation: Rotation Speed", rotSpeed);
        packet.put("Rotation: Current Rotation", drive.getRotation());
        packet.put("Rotation: New Rotation", newRotation);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0,0,0,false);
    }


    @Override
    public boolean isFinished() {
        double distanceToNewPose = Math.abs(drive.getRotation()
                - newRotation);
        return distanceToNewPose <= tolerance;
    }
}