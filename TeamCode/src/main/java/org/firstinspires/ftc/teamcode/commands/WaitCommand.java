package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class WaitCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


    //Initializes some important values
    private final double waitTime;
    private double endTime = -1;

    /**
     * Makes the bot stop for a specified period
     */
    public WaitCommand(double waitTime) {
        this.waitTime = waitTime;

    }


    @Override
    public void initialize() {
        this.endTime = System.currentTimeMillis() / 1000.0 + waitTime;
    }


    @Override
    public boolean isFinished() {
        return endTime <= System.currentTimeMillis() / 1000.0 && isScheduled();
    }
}