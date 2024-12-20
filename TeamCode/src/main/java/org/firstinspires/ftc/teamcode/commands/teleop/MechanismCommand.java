package org.firstinspires.ftc.teamcode.commands.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabberMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystemWPid;

public class MechanismCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final PivotMechanismSubsystem pivot;
    private final ExtensionMechanismSubsystem extension;
    private final GrabberMechanismSubsystem grabber;

    //Make a gamepad named gamepad1
    private final Gamepad gamepad2;
    //private Telemetry telemetry;
    private final TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    //TODO: Correct These Values
    //Initializes some important values
    private final double maxTranslationSpeed = Configuration.maxTranslationSpeed;
    private final double maxRotSpeed = 2 * Math.PI;

    /**
     * Creates a new Command To interface with the mechansisms.
     *
     *  @param pivot
     *  @param extension
     *  @param grabber
     *  @param gamepad2
     *  The subsystems and gamepad used by this command.
     *  @since 11/12/24
     */


    public MechanismCommand(PivotMechanismSubsystem pivot,
                            ExtensionMechanismSubsystem extension,
                            GrabberMechanismSubsystem grabber, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        this.pivot = pivot;
        this.extension = extension;
        this.grabber = grabber;

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(pivot, extension, grabber);
    }


    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        double pivotPower = (gamepad2.right_trigger - gamepad2.left_trigger * 0.5) * 0.5;

        pivot.setPivotPower(pivotPower);

        // The Extension code used to control arm used to extend and such
        if (gamepad2.right_bumper) {
            extension.setExtensionPower(1);
        } else if (gamepad2.left_bumper) {
            extension.setExtensionPower(-1);
        } else {
            extension.setExtensionPower(0);
        }

        // Grabber Code
        if (gamepad2.b) {
            // Pushes The Stuff out
            grabber.setGrabberPower(0.5);
        } else if (gamepad2.y) {
            // Pulls The Stuff In
            grabber.setGrabberPower(-1);
        } else {
            // Makes sure it's not active most of the time unless you need to use it.
            grabber.setGrabberPower(0);
        }
        packet.put("Extension: Current Revolutions",
                extension.getExtensionRevolutions());


        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
