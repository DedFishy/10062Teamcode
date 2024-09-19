package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {


    //Mecanum Drive Stuff
    private MecanumDriveKinematics m_kinematics;

    //Robot Drivetrain and gyroscope
    private BNO055IMU imu;
    private Motor fr_drive;
    private Motor br_drive;
    private Motor bl_drive;
    private Motor fl_drive;
    /**
     * Creates our drive subsystem
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        //Robot drivetrain and gyroscope initialization
        //TODO: Get motor brand and type were using this year!
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Motor fr_drive = new Motor(hardwareMap, "fr_drive");
        Motor fl_drive = new Motor(hardwareMap, "fl_drive");
        Motor br_drive = new Motor(hardwareMap, "br_drive");
        Motor bl_drive = new Motor(hardwareMap, "bl_drive");

        // Locations of the wheels relative to the robot center.

        //TODO:  Change to real values when we have it
        Translation2d m_frontLeftLocation =
                new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation =
                new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation =
                new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation =
                new Translation2d(-0.381, -0.381);

        // Creating my kinematics object using the wheel locations.
        m_kinematics = new MecanumDriveKinematics
                (
                        m_frontLeftLocation, m_frontRightLocation,
                        m_backLeftLocation, m_backRightLocation
                );

        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
        MecanumDriveOdometry m_odometry = new MecanumDriveOdometry
                (
                        m_kinematics, getGyroHeading(),
                        new Pose2d(5.0, 13.5, new Rotation2d()
                        )
                );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private Rotation2d getGyroHeading() {

        Rotation2d rot2d;
        rot2d = new Rotation2d(imu.getAngularOrientation().firstAngle);
        return rot2d;
    }
}
