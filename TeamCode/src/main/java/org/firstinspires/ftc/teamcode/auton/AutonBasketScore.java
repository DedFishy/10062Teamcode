package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.auton.AutonDriveRotDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.auton.AutonDriveYDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.auton.AutonExtend;
import org.firstinspires.ftc.teamcode.commands.auton.AutonGrabberRelease;
import org.firstinspires.ftc.teamcode.commands.auton.AutonPivotLower;
import org.firstinspires.ftc.teamcode.commands.auton.AutonPivotRaise;
import org.firstinspires.ftc.teamcode.commands.auton.AutonRetract;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabberMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystem;

@Autonomous(name = "Basket Score", preselectTeleOp = "TeleopCommandBased", group = "2024-2025")
public class AutonBasketScore extends CommandOpMode {

    private SequentialCommandGroup autonCommandGroup;
    private DriveSubsystem drive;
    private PivotMechanismSubsystem pivot;
    private ExtensionMechanismSubsystem extension;
    private GrabberMechanismSubsystem grabber;


    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        pivot = new PivotMechanismSubsystem(hardwareMap);
        extension = new ExtensionMechanismSubsystem(hardwareMap);
        grabber = new GrabberMechanismSubsystem(hardwareMap);

        this.autonCommandGroup = new SequentialCommandGroup(
                new AutonDriveYDistanceCommand(drive, 1, 0.5),
                new AutonDriveRotDistanceCommand(drive, 2.35619),
                new AutonDriveYDistanceCommand(drive, 1, 0.5),
                new AutonPivotRaise(pivot, 0.1),
                new AutonExtend(extension, 2),
                new AutonGrabberRelease(grabber),
                new AutonRetract(extension, 2),
                new AutonPivotLower(pivot, 0.25)
        );

        schedule(autonCommandGroup);

    }
}
