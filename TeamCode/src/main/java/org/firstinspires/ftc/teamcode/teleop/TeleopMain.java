package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "2023MainOp (Main)")
@Disabled

public class TeleopMain extends LinearOpMode {

    private DcMotor fr_drive;
    private DcMotor br_drive;
    private DcMotor bl_drive;
    private DcMotor fl_drive;
    private BNO055IMU imu;

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
    public void runOpMode() {
        BNO055IMU.Parameters imuParameters;
        double TopSpeed;
        double MaxTurnSpeed;
        double Correction;
        int HoldDirection;
        double Pcorrection;
        Orientation angles;
        Acceleration gravity;
        float LeftStickX;
        float LeftStickY;
        float RightStickx;
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

        fr_drive = hardwareMap.get(DcMotor.class, "fr_drive");
        br_drive = hardwareMap.get(DcMotor.class, "br_drive");
        bl_drive = hardwareMap.get(DcMotor.class, "bl_drive");
        fl_drive = hardwareMap.get(DcMotor.class, "fl_drive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

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
        Pcorrection = 0.015;
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Get absolute orientation
                // Get acceleration due to force of gravity.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                LeftStickX = gamepad1.left_stick_x;
                LeftStickY = -gamepad1.left_stick_y;
                RightStickx = gamepad1.right_stick_x;
                StickHypot = Math.sqrt(LeftStickX * LeftStickX + LeftStickY * LeftStickY);
                if (gamepad1.start) {
                    imu.initialize(imuParameters);
                    HoldDirection = 0;
                }
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
                // Display gravitational acceleration.
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */

}
