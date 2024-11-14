package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *
 *
 **/
public class DriveSubsystem extends SubsystemBase {


    double TopSpeed;
    double MaxTurnSpeed;
    double Correction;
    int HoldDirection;
    double Pcorrection;
    Orientation angles;
    Acceleration gravity;
    double LeftStickX;
    double LeftStickY;
    double RightStickx;
    double StickHypot;
    float RobotAngle;
    double IntStickAngle;
    double StickAngle;
    double DriveAngle;
    double FwdVect;
    double Turn;
    double RightSlideVect;
    double FLint;
    double RLint;
    double FRint;
    double RRint;
    double ABS_FL;
    double ABS_FR;
    double ABS_RL;
    double ABS_RR;
    double MaxDrive;
    double DriveFL;
    double DriveFR;
    double DriveRL;
    double DriveRR;
    private DcMotor fr_drive;
    private DcMotor br_drive;
    private DcMotor bl_drive;
    private DcMotor fl_drive;
    private BNO055IMU imu;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private BNO055IMU.Parameters imuParameters;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private double CPR = 537.0;
    private double translationX = 0;
    private double translationY = 0;
    private double translationRot = 0;

    private double fr_motor_offset;
    private double fl_motor_offset;
    private double br_motor_offset;
    private double bl_motor_offset;
    private double imu_offset = 0;

    public DriveSubsystem (HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.packet = packet;
        this.dashboard = dashboard;

        fr_drive = hardwareMap.get(DcMotor.class, "fr_drive");
        br_drive = hardwareMap.get(DcMotor.class, "br_drive");
        bl_drive = hardwareMap.get(DcMotor.class, "bl_drive");
        fl_drive = hardwareMap.get(DcMotor.class, "fl_drive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fr_drive.getCurrentPosition();
        fr_drive.setDirection(DcMotor.Direction.FORWARD);
        br_drive.setDirection(DcMotor.Direction.FORWARD);
        bl_drive.setDirection(DcMotor.Direction.REVERSE);
        fl_drive.setDirection(DcMotor.Direction.REVERSE);
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Prompt user to press start buton.
        telemetry.addData("IMU Example", "Press start to continue...");
        TopSpeed = 0.68;
        MaxTurnSpeed = 0.5;
        Correction = 0.25;
        HoldDirection = 0;
        Pcorrection = 0.01;
        telemetry.update();

    }



    /**
     * Describe this function...
     */
    private double powerLimit(double motorPower, double limit) {
        return limit * motorPower;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void periodic() {
        // Get absolute orientation
        // Get acceleration due to force of gravity.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        LeftStickX = translationX;
        LeftStickY = translationY;
        RightStickx = translationRot;
        StickHypot = Math.sqrt(LeftStickX * LeftStickX + LeftStickY * LeftStickY);
        RobotAngle = -angles.firstAngle;
        if (StickHypot == 0) {
            IntStickAngle = 0;
        } else {
            IntStickAngle = Math.acos(LeftStickY / StickHypot) / Math.PI * 180;
        }
        if (LeftStickX > 0) {
            StickAngle = IntStickAngle;
        } else {
            StickAngle = -IntStickAngle;
        }
        DriveAngle = StickAngle - RobotAngle;
        FwdVect = Math.cos(DriveAngle / 180 * Math.PI);
        if (StickHypot > 1) {
            StickHypot = 1;
        }
        if (Correction == 0) {
            HoldDirection = (int) RobotAngle;
        }
        if (RightStickx == 0) {
            Correction = 1;
        } else {
            Correction = 0;
        }
        if (Correction == 0) {
            Turn = RightStickx * MaxTurnSpeed;
        } else {
            if (HoldDirection - RobotAngle > 180) {
                Turn = ((HoldDirection - RobotAngle) - 360) * Pcorrection;
            } else if (HoldDirection - RobotAngle < -180) {
                Turn = ((HoldDirection - RobotAngle) + 360) * Pcorrection;
            } else {
                Turn = (HoldDirection - RobotAngle) * Pcorrection;
            }
        }
        RightSlideVect = Math.sin(DriveAngle / 180 * Math.PI);
        FLint = FwdVect + RightSlideVect + Turn;
        RLint = FwdVect + (Turn - RightSlideVect);
        FRint = FwdVect - (RightSlideVect + Turn);
        RRint = FwdVect + (RightSlideVect - Turn);
        ABS_FL = Math.abs(FLint);
        ABS_FR = Math.abs(FRint);
        ABS_RL = Math.abs(RLint);
        ABS_RR = Math.abs(RRint);
        MaxDrive = ABS_FL;
        if (MaxDrive < ABS_FR) {
            MaxDrive = ABS_FR;
        }
        if (MaxDrive < ABS_RL) {
            MaxDrive = ABS_RL;
        }
        if (MaxDrive < ABS_RR) {
            MaxDrive = ABS_RR;
        }
        DriveFL = (FLint / MaxDrive) * StickHypot * TopSpeed;
        if (StickHypot == 0) {
            DriveFL = Turn;
        }
        DriveFR = (FRint / MaxDrive) * StickHypot * TopSpeed;
        if (StickHypot == 0) {
            DriveFR = -Turn;
        }
        DriveRL = (RLint / MaxDrive) * StickHypot * TopSpeed;
        if (StickHypot == 0) {
            DriveRL = Turn;
        }
        DriveRR = (RRint / MaxDrive) * StickHypot * TopSpeed;
        if (StickHypot == 0) {
            DriveRR = -Turn;
        }


        telemetry.addData("Robot Angle", RobotAngle);
        telemetry.addData("StickAngle", StickAngle);
        telemetry.addData("DriveAngle", DriveAngle);
        telemetry.addData("Turn", Turn);
        telemetry.addData("FwdVect", FwdVect);
        telemetry.addData("SlideVect", RightSlideVect);
        telemetry.addData("FrontRight", DriveFR);
        telemetry.addData("FrontLeft", DriveFL);
        telemetry.addData("Rear Right", DriveRR);
        telemetry.addData("Rear Left", DriveRL);
        telemetry.addData("MaxDrive", MaxDrive);
        bl_drive.setPower(DriveRL);
        fr_drive.setPower(DriveFR);
        fl_drive.setPower(DriveFL);
        br_drive.setPower(DriveRR);

        // FTC dashboard

        //packet.put("Robot Angle", RobotAngle);
        //packet.put("Stick Angle", StickAngle);
        //packet.put("Drive Angle", DriveAngle);
        //packet.put("Turn", Turn);
        //packet.put("Fwd Vect", FwdVect);
        //packet.put("SlideVect", RightSlideVect);
        //packet.put("Front Right Speed", DriveFR);
        //packet.put("Front Left Speed", DriveFL);
        //packet.put("Rear Right Speed", DriveRR);
        //packet.put("Rear Left Speed", DriveRL);
        //packet.put("Max Drive", MaxDrive);
        //packet.put("Front Right Encoder", fr_drive.getCurrentPosition());
        //packet.put("Front Left Encoder", -fl_drive.getCurrentPosition());
        //packet.put("Rear Right Encoder", br_drive.getCurrentPosition());
        //packet.put("Rear Left Encoder", bl_drive.getCurrentPosition());
        //packet.put("Distance", getDistance());
        //dashboard.sendTelemetryPacket(packet);

        // Display gravitational acceleration.
        telemetry.update();
    }

    public void drive(double x_speed, double y_speed, double rot_speed,
                      double maxTranslationSpeed, boolean fieldRelative) {
        translationX = x_speed;
        translationY = y_speed;
        translationRot = rot_speed;
    }


    public double getDistance() {
        double fr_encoder_current_position_with_offset = fr_drive.getCurrentPosition() -
                fr_motor_offset;
        double fl_encoder_current_position_with_offset = fl_drive.getCurrentPosition() -
                fl_motor_offset;
        double br_encoder_current_position_with_offset = br_drive.getCurrentPosition() -
                br_motor_offset;
        double bl_encoder_current_position_with_offset = bl_drive.getCurrentPosition() -
                bl_motor_offset;

        double average = ((fr_encoder_current_position_with_offset +
                -fl_encoder_current_position_with_offset +
                br_encoder_current_position_with_offset +
                bl_encoder_current_position_with_offset) / 4) / CPR;
        return -average;
    }

    public void clearEncoderPulse() {
        fr_motor_offset = fr_drive.getCurrentPosition();
        fl_motor_offset = fl_drive.getCurrentPosition();
        br_motor_offset =  br_drive.getCurrentPosition();
        bl_motor_offset = bl_drive.getCurrentPosition();
    }
    /**
     *  Resets The imu
     **/

    public void resetImu() {
        imu.initialize(imuParameters);
        imu_offset = imu.getAngularOrientation().firstAngle;
        HoldDirection = 0;
    }
    /**
     *  Get's The imu's current rotation
     **/
    public double getRotation() {
        return imu.getAngularOrientation().firstAngle - imu_offset;
    }

}
