package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.auton.AutonDriveRotDistanceCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Auton Command Based")
public class AutonCommandBased extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private AutonDriveRotDistanceCommand driveDistance;

    @Override
    public void initialize() {

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        driveDistance = new AutonDriveRotDistanceCommand(driveSubsystem,180);

        register(driveSubsystem);
        schedule(driveDistance);


    }
}
