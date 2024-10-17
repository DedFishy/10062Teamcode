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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class DriveSubsystem extends SubsystemBase {


    //Internal Stuff.   TOUCH WITH CARE!
    private MecanumDriveKinematics m_kinematics;
    private MecanumDriveOdometry m_odometry;
    private Pose2d m_pose;

    private double fr_motor_offset;
    private double fl_motor_offset;
    private double br_motor_offset;
    private double bl_motor_offset;
    private double wheelCircumference = 0.104 * Math.PI; //Centimeters = 30.6

    //Robot Drivetrain and Gyroscope
    private final BNO055IMU imu;
    private final Motor fr_drive;
    private final Motor br_drive;
    private final Motor bl_drive;
    private final Motor fl_drive;
    private final Telemetry telemetry;
    /**
     * Creates our drive subsystem
     */
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        //Robot drivetrain and gyroscope initialization
        //TODO: Get motor brand and type were using this year!
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        fr_drive = new Motor(hardwareMap, "fr_drive", Motor.GoBILDA.RPM_312);
        fl_drive = new Motor(hardwareMap, "fl_drive", Motor.GoBILDA.RPM_312);
        br_drive = new Motor(hardwareMap, "br_drive", Motor.GoBILDA.RPM_312);
        bl_drive = new Motor(hardwareMap, "bl_drive", Motor.GoBILDA.RPM_312);
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();

        imu.initialize(imuParams);

        fl_drive.setInverted(true);
        bl_drive.setInverted(true);

        m_pose = new Pose2d();

        this.telemetry = telemetry;
        /** The counts per revolution of the motor as well as the distance per pulse.
         *  AND WHAT IS WRONG WITH THE VARIABLE TYPES??!?!?!?
         */
        final double CPR = fr_drive.getCPR();
        //final double wheelCircumference = 0.104 * Math.PI; //Centimeters = 30.6
        //final double DPP = wheelCircumference / CPR;
        //fr_drive.setDistancePerPulse(DPP);
        //fl_drive.setDistancePerPulse(DPP);
        //br_drive.setDistancePerPulse(DPP);
        //bl_drive.setDistancePerPulse(DPP);
        // Locations of the wheels relative to the robot center.
        // x offset is 7.5
        // y offset is 6.75
        Translation2d m_frontLeftLocation =
                new Translation2d(0.1905, 0.17145);
        Translation2d m_frontRightLocation =
                new Translation2d(0.1905, -0.17145);
        Translation2d m_backLeftLocation =
                new Translation2d(-0.1905, 0.17145);
        Translation2d m_backRightLocation =
                new Translation2d(-0.1905, -0.17145);

        // Creating my kinematics object using the wheel locations.
        m_kinematics = new MecanumDriveKinematics
                (
                        m_frontLeftLocation, m_frontRightLocation,
                        m_backLeftLocation, m_backRightLocation
                );

        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
        //m_odometry = new MecanumDriveOdometry
        //        (
        //                m_kinematics, getGyroHeading(),
        //                new Pose2d(0.0, 0.0, new Rotation2d()
        //                )
        //        );


    }

    @Override
    public void periodic() {
        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (
                        fl_drive.encoder.getRate(), fr_drive.encoder.getRate(),
                        bl_drive.encoder.getRate(), br_drive.encoder.getRate()
                );

        // Get my gyro angle.
        Rotation2d gyroAngle = getGyroHeading();

        // Update the pose
        double unixTime = System.currentTimeMillis() / 1000.0;
        m_pose = m_odometry.updateWithTime(unixTime, gyroAngle, wheelSpeeds);
    }

    private Rotation2d getGyroHeading() {

        Rotation2d rot2d;
        rot2d = new Rotation2d(imu.getAngularOrientation().firstAngle + Math.toRadians(90));
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
        br_drive.set(wheelSpeeds.rearRightMetersPerSecond / maxTranslationSpeed);
        telemetry.addData("X Speed", x_speed);
        telemetry.addData("Y Speed", y_speed);
        telemetry.addData("Rotation Speed", rot_speed);
        telemetry.addData("Front Left Wheel Speed", wheelSpeeds.frontLeftMetersPerSecond);
        telemetry.addData("Front Right Wheel Speed", wheelSpeeds.frontRightMetersPerSecond);
        telemetry.addData("Back Left Wheel Speed", wheelSpeeds.rearLeftMetersPerSecond);
        telemetry.addData("Back Right Wheel Speed", wheelSpeeds.rearRightMetersPerSecond);
        telemetry.addData("Front Right Encoder Revolutions", Math.abs(fr_drive.encoder.getRevolutions()));
        telemetry.addData("Current Pose Y", m_pose.getY());
        telemetry.addData("Current Pose X", m_pose.getX());
        telemetry.addData("Current Pose Rotation", m_pose.getRotation().getDegrees());
        telemetry.update();
    }
    public double encoderRevolutions(){
        telemetry.addData("Encoder Revolutions", Math.abs(fr_drive.encoder.getRevolutions()));
        return Math.abs(fr_drive.encoder.getRevolutions());
    }

    /**
     * Our own encoder system
     */

    public double getDistance() {
        double fr_encoder_current_position_with_offset = fr_drive.encoder.getRevolutions() -
                fr_motor_offset;
        double fl_encoder_current_position_with_offset = fl_drive.encoder.getRevolutions() -
                fl_motor_offset;
        double br_encoder_current_position_with_offset = br_drive.encoder.getRevolutions() -
                br_motor_offset;
        double bl_encoder_current_position_with_offset = bl_drive.encoder.getRevolutions() -
                bl_motor_offset;

        double average = ((fr_encoder_current_position_with_offset +
                fl_encoder_current_position_with_offset + br_encoder_current_position_with_offset +
                bl_encoder_current_position_with_offset) / 4) * wheelCircumference;
        return average;
    }

    public void clearEncoderPulse() {
        fr_motor_offset = fr_drive.encoder.getRevolutions();
        fl_motor_offset = fl_drive.encoder.getRevolutions();
        br_motor_offset =  br_drive.encoder.getRevolutions();
        bl_motor_offset = bl_drive.encoder.getRevolutions();
    }
}
