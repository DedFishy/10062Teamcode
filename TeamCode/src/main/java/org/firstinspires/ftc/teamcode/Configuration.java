package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Configuration {
    public static volatile PIDCoefficients pidCoefficients =
            new PIDCoefficients(0.5,0,0);

    public static boolean TeleopFieldRelative = false;
    public static boolean AutonFieldRelative = false;
    public static volatile double maxTranslationSpeed = 1;
}
