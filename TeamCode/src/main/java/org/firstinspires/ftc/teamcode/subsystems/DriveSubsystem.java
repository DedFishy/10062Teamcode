package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {


    //Internal Stuff.   TOUCH WITH CARE!
    private MecanumDriveKinematics m_kinematics;
    private MecanumDriveOdometry m_odometry;
    private Pose2d m_pose;

    //Robot Drivetrain and Gyroscope
    private final BNO055IMU imu;
    private final Motor fr_drive;
    private final Motor br_drive;
    private final Motor bl_drive;
    private final Motor fl_drive;
    /**
     * Creates our drive subsystem
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        //Robot drivetrain and gyroscope initialization
        //TODO: Get motor brand and type were using this year!
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        fr_drive = new Motor(hardwareMap, "fr_drive");
        fl_drive = new Motor(hardwareMap, "fl_drive");
        br_drive = new Motor(hardwareMap, "br_drive");
        bl_drive = new Motor(hardwareMap, "bl_drive");
        //x offset is 7.5
        //y offset is 6.75


        /* The counts per revolution of the motor as well as the distance per pulse.
         *  AND WHAT IS WRONG WITH THE VARIABLE TYPES??!?!?!?
         */
        final double CPR = fr_drive.getCPR();
        final double wheelCircumference = 0.02 * 2 * Math.PI;
        final double DPP = wheelCircumference / CPR;
        fr_drive.setDistancePerPulse(DPP);
        fl_drive.setDistancePerPulse(DPP);
        br_drive.setDistancePerPulse(DPP);
        bl_drive.setDistancePerPulse(DPP);

        // Locations of the wheels relative to the robot center.
        //TODO:  Change to real values once we have it
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
        m_odometry = new MecanumDriveOdometry
                (
                        m_kinematics, getGyroHeading(),
                        new Pose2d(5.0, 13.5, new Rotation2d()
                        )
                );
    }

    @Override
    public void periodic() {
        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (
                        fl_drive.getRate(), fr_drive.getRate(),
                        bl_drive.getRate(), br_drive.getRate()
                );

        // Get my gyro angle.
        Rotation2d gyroAngle = getGyroHeading();
        // Update the pose

        double unixTime = System.currentTimeMillis() / 1000.0;
        m_pose = m_odometry.updateWithTime(unixTime, gyroAngle, wheelSpeeds);
    }

    private Rotation2d getGyroHeading() {

        Rotation2d rot2d;
        rot2d = new Rotation2d(imu.getAngularOrientation().firstAngle);
        return rot2d;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public void drive(double x_speed, double y_speed, double rot_speed, double maxTranslationSpeed) {
        // The desired field relative speed here is 2 meters per second
        // toward the opponent's alliance station wall, and 2 meters per
        // second toward the left field boundary. The desired rotation
        // is a quarter of a rotation per second counterclockwise.
        // The current robot angle is 45 degrees.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot_speed, getGyroHeading()
        );

        // Now use this in our kinematics
        MecanumDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(speeds);


        fl_drive.set(wheelSpeeds.frontLeftMetersPerSecond / maxTranslationSpeed);
        fr_drive.set(wheelSpeeds.frontRightMetersPerSecond / maxTranslationSpeed);
        bl_drive.set(wheelSpeeds.rearLeftMetersPerSecond / maxTranslationSpeed);
        bl_drive.set(wheelSpeeds.rearRightMetersPerSecond / maxTranslationSpeed);
    }
}
