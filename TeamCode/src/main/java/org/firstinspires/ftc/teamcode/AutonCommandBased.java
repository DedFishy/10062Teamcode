package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.teleop.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Auton Command Based")
public class AutonCommandBased extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        driveCommand = new DriveCommand(driveSubsystem, gamepad1);

        register(driveSubsystem);
    }
}
