package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.teleop.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.teleop.MechanismCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabberMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystem;

@TeleOp(name = "Teleop Command Based")
//@Disabled
public class TeleopCommandBased extends CommandOpMode {


    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private MechanismCommand mechanismCommand;
    private PivotMechanismSubsystem pivot;
    private ExtensionMechanismSubsystem extension;
    private GrabberMechanismSubsystem grabber;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        pivot = new PivotMechanismSubsystem(hardwareMap);
        extension = new ExtensionMechanismSubsystem(hardwareMap);
        grabber = new GrabberMechanismSubsystem(hardwareMap);


        // Register The Commands
        driveCommand = new DriveCommand(driveSubsystem, gamepad1);
        mechanismCommand = new MechanismCommand(pivot, extension, grabber, gamepad2);

        driveSubsystem.setDefaultCommand(driveCommand);
        register(driveSubsystem, pivot, extension, grabber);
        driveCommand.schedule();
        mechanismCommand.schedule();
    }


}
