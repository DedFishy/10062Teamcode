package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

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

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(pivot, extension, grabber);
    }


    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        double pivotPower = (gamepad2.right_trigger - gamepad2.left_trigger * 0.5) * 0.6;

        pivot.setPivotPower(pivotPower);

        // The Extension code used to control arm used to extend and such
        if (gamepad2.right_bumper) {
            extension.setExtensionPower(.5);
        } else if (gamepad2.left_bumper) {
            extension.setExtensionPower(-.5);
        } else {
            extension.setExtensionPower(0);
        }

        // Grabber Code
        if (gamepad2.b) {
            // Pulls The Suff In
            grabber.setGrabberPower(1);
        } else if (gamepad2.y) {
            // Pushes The Stuff out
            grabber.setGrabberPower(-0.5);
        } else {
            // Makes sure it's not active most of the time unless you need to use it.
            grabber.setGrabberPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
