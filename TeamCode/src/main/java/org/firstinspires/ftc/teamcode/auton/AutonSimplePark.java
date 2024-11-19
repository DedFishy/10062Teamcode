package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.auton.AutonDriveYDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.auton.OffsetImu;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Auton Simple Park", preselectTeleOp = "Teleop Command Based")
public class AutonSimplePark extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private AutonDriveYDistanceCommand driveDistance;

    private SequentialCommandGroup autonCommandGroup;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        driveDistance = new AutonDriveYDistanceCommand(driveSubsystem, 1.5,0.5);

        autonCommandGroup = new SequentialCommandGroup(new WaitCommand(0),
                driveDistance, new OffsetImu(driveSubsystem, 90));

        register(driveSubsystem);
        schedule(autonCommandGroup);
    }
}