package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.auton.AutonDriveYDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.auton.DriveCircumference;
import org.firstinspires.ftc.teamcode.commands.teleop.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Auton Command Based")
public class AutonCommandBased extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private AutonDriveYDistanceCommand driveDistance;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        driveDistance = new AutonDriveYDistanceCommand(driveSubsystem,1,0.2);

        register(driveSubsystem);
        schedule(driveDistance);
    }
}
