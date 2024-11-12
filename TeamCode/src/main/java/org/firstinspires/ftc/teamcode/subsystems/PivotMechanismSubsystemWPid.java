package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration;

public class PivotMechanismSubsystemWPid extends SubsystemBase {

    private MotorEx pivotMotor;
    private double setPoint;
    private final TelemetryPacket packet = new TelemetryPacket();

    public PivotMechanismSubsystemWPid(HardwareMap hardwareMap) {
        this.pivotMotor = new MotorEx(hardwareMap, "pivot", Motor.GoBILDA.RPM_435);
        pivotMotor.setRunMode(Motor.RunMode.PositionControl);
        this.pivotMotor.setPositionCoefficient(Configuration.pidCoefficients.p);
        this.pivotMotor.setFeedforwardCoefficients(0,0,0);
        pivotMotor.setInverted(false);
    }

    public void setDesiredPivotEncoderPulses (double setPoint)  {
        this.setPoint = setPoint;
        pivotMotor.set(setPoint);
    }
    public double getSetPoint() {
        return this.setPoint;
    }

    @Override
    public void periodic() {
        updatePIDCoefficients();
        packet.put("Pivot: Set Point", getSetPoint());
        packet.put("Pivot: P Value", pivotMotor.getPositionCoefficient());
        packet.put("Pivot: Current Position", pivotMotor.getCurrentPosition());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }
    public void updatePIDCoefficients() {
        double kp = Configuration.pidCoefficients.p;

        pivotMotor.setPositionCoefficient(kp);
    }
}
