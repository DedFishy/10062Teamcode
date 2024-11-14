package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * The command to drive along the y axis
 * Rotation Auton Command {@link org.firstinspires.ftc.teamcode.commands.auton.AutonDriveRotDistanceCommand}
 * Drive along the Y axis command {@link org.firstinspires.ftc.teamcode.commands.auton.AutonDriveYDistanceCommand}
 *
 * @author evokerking1, Mentors (DedFishy, Jetvac2)
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