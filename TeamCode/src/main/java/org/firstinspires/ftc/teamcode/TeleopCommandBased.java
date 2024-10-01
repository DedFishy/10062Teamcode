package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.teleop.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "Teleop Command Based")
public class TeleopCommandBased extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveCommand = new DriveCommand(driveSubsystem, gamepad1);
        driveSubsystem.setDefaultCommand(driveCommand);
        register(driveSubsystem);
    }
}
