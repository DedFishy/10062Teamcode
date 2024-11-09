package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GrabberMechanismSubsystem extends SubsystemBase {
    private CRServo grabber;
    public GrabberMechanismSubsystem(HardwareMap hardwareMap) {
        this.grabber = new CRServo(hardwareMap, "grabber");
        grabber.setRunMode(Motor.RunMode.RawPower);
        grabber.setInverted(true);
    }

    public void setGrabberPower(double power) {
        grabber.set(power);
    }
}
