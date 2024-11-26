package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotMechanismSubsystem extends SubsystemBase {

    private MotorEx pivotMotor;

    public PivotMechanismSubsystem(HardwareMap hardwareMap) {
        this.pivotMotor = new MotorEx(hardwareMap, "pivot", Motor.GoBILDA.RPM_435);
        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.setInverted(true);
    }

    public void setPivotPower(double power) {
        pivotMotor.set(power);

    }

    public double getPivotPosition() {
        return pivotMotor.encoder.getRevolutions();
    }
}
