package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.auton.AutonDriveRotDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.auton.AutonDriveYDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.auton.AutonExtend;
import org.firstinspires.ftc.teamcode.commands.auton.AutonGrabberRelease;
import org.firstinspires.ftc.teamcode.commands.auton.AutonPivotLower;
import org.firstinspires.ftc.teamcode.commands.auton.AutonPivotRaise;
import org.firstinspires.ftc.teamcode.commands.auton.AutonRetract;
import org.firstinspires.ftc.teamcode.commands.auton.OffsetImu;
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
                new ParallelRaceGroup(new AutonPivotRaise(pivot), new WaitCommand(5000)),
                new ParallelRaceGroup(new AutonExtend(extension), new WaitCommand(2000)),
                new AutonDriveYDistanceCommand(drive, 1.1, 0.5),
                new ParallelRaceGroup(new AutonGrabberRelease(grabber), new WaitCommand(5000)),
                new ParallelRaceGroup(new AutonRetract(extension), new WaitCommand(3000)),
                new ParallelRaceGroup(new AutonPivotLower(pivot), new WaitCommand(1000),
                new OffsetImu(drive, -90))
        );

        schedule(autonCommandGroup);

    }
}