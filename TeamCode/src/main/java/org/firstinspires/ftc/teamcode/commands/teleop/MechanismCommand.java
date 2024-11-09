package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabberMechanismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotMechanismSubsystem;

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
     * Creates a new ExampleCommand.
     *
     *  @param pivot
     *  @param extension
     *  @param grabber
     *  @param gamepad2
     *  The subsystem used by this command.
     */

    public MechanismCommand(PivotMechanismSubsystem pivot,
                            ExtensionMechanismSubsystem extension,
                            GrabberMechanismSubsystem grabber,Gamepad gamepad2) {
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
        double pivotPower = gamepad2.right_trigger - gamepad2.left_trigger * 0.5;

        pivot.setPivotPower(pivotPower);

        if (gamepad2.right_bumper) {
            extension.setExtensionPower(1);
        } else if (gamepad2.left_bumper) {
            extension.setExtensionPower(-1);
        } else {
            extension.setExtensionPower(0);
        }
        if (gamepad2.b) {
            grabber.setGrabberPower(1);
        } else if (gamepad2.y) {
            grabber.setGrabberPower(-1);
        } else {
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
