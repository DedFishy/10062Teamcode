package org.firstinspires.ftc.teamcode.commands.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * The teleop drive command that uses the drive subsystem
 */
public class DriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_DriveSubsystem;

    //Make a gamepad named gamepad1
    private final Gamepad gamepad1;

    //TODO: Correct These Values
    //Initializes some important values
    private final double maxTranslationSpeed = 0.5;
    private final double maxRotSpeed = 2 * Math.PI;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveCommand(DriveSubsystem subsystem, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        m_DriveSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_DriveSubsystem);
    }


    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        m_DriveSubsystem.drive(
                gamepad1.left_stick_x * maxTranslationSpeed,
                -gamepad1.left_stick_y * 0 * maxTranslationSpeed,
                gamepad1.right_stick_x  * maxRotSpeed, maxTranslationSpeed,
                false);


        if(gamepad1.dpad_up) {
            m_DriveSubsystem.clearEncoderPulse();
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