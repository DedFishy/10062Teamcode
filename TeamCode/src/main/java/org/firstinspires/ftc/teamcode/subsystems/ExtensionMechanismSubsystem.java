package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtensionMechanismSubsystem extends SubsystemBase {
    private MotorEx extensionMechanism;
    public ExtensionMechanismSubsystem(HardwareMap hardwareMap) {
        this.extensionMechanism = new MotorEx(hardwareMap, "extension", Motor.GoBILDA.RPM_435);
        extensionMechanism.setRunMode(Motor.RunMode.RawPower);
        extensionMechanism.setInverted(true);
    }

    public void setExtensionPower(double power) {
        extensionMechanism.set(power);
    }
}
