package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * A sleep command
 */
public class WaitTimer {

    //Initializes some important values
    private final double waitTime;
    private double endTime = -1;

    /**
     * Makes the bot stop for a specified period
     */
    public WaitTimer(double waitTime) {
        this.waitTime = waitTime;

    }


    public void startTimer() {
        this.endTime = System.currentTimeMillis() / 1000.0 + waitTime;
    }


    public boolean isFinished() {
        return endTime <= System.currentTimeMillis() / 1000.0 && endTime != -1;
    }
}