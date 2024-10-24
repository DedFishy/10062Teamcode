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

    //Pidf Controller init
    private final double kp = 1;
    private final double ki = 0;
    private final double kd = 0;

    //PIDFController fr_drive_pidf = new PIDFController(0,0,0,0);
    //PIDFController fl_drive_pidf = new PIDFController(0,0,0,0);
    //PIDFController br_drive_pidf = new PIDFController(0,0,0,0);
    //PIDFController bl_drive_pidf = new PIDFController(0,0,0,0);

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

        //Configures the internal pid loop system

        fl_drive.setVeloCoefficients(kp,ki,kd);
        fr_drive.setVeloCoefficients(kp,ki,kd);
        bl_drive.setVeloCoefficients(kp,ki,kd);
        br_drive.setVeloCoefficients(kp,ki,kd);

        fl_drive.setRunMode(Motor.RunMode.VelocityControl);
        fr_drive.setRunMode(Motor.RunMode.VelocityControl);
        bl_drive.setRunMode(Motor.RunMode.VelocityControl);
        br_drive.setRunMode(Motor.RunMode.VelocityControl);



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
        //m_pose = m_odometry.updateWithTime(unixTime, gyroAngle, wheelSpeeds);
    }

    private Rotation2d getGyroHeading() {

        Rotation2d rot2d;
        rot2d = new Rotation2d(imu.getAngularOrientation().firstAngle + Math.toRadians(90));
        return rot2d;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public void drive(double x_speed, double y_speed, double rot_speed,
                      double maxTranslationSpeed, boolean fieldRelative) {
        // The desired field relative speed here is 2 meters per second
        // toward the opponent's alliance station wall, and 2 meters per
        // second toward the left field boundary. The desired rotation
        // is a quarter of a rotation per second counterclockwise.
        // The current robot angle is 45 degrees.
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x_speed, y_speed, rot_speed, getGyroHeading()
            );
        } else {
            speeds = new ChassisSpeeds(
                    x_speed, y_speed, rot_speed
            );
        }



        // Now use this in our kinematics
        MecanumDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(speeds);


        //Converting MPS to CPS
        double fl_desiredWheelSpeedCPS = (wheelSpeeds.frontLeftMetersPerSecond /
                wheelCircumference) / this.fl_drive.getCPR();

        double fr_desiredWheelSpeedCPS = (wheelSpeeds.frontRightMetersPerSecond /
                wheelCircumference) / this.fr_drive.getCPR();

        double bl_desiredWheelSpeedCPS = (wheelSpeeds.rearLeftMetersPerSecond /
                wheelCircumference) / this.fl_drive.getCPR();

        double br_desiredWheelSpeedCPS = (wheelSpeeds.rearRightMetersPerSecond /
                wheelCircumference) / this.br_drive.getCPR();

        fl_drive.set(fl_desiredWheelSpeedCPS);
        fr_drive.set(fr_desiredWheelSpeedCPS);
        bl_drive.set(bl_desiredWheelSpeedCPS);
        br_drive.set(br_desiredWheelSpeedCPS);

        //fl_drive.set(1 * (wheelSpeeds.frontLeftMetersPerSecond / maxTranslationSpeed));
        //fr_drive.set(1 * (wheelSpeeds.frontRightMetersPerSecond / maxTranslationSpeed));
        //bl_drive.set(1 * (wheelSpeeds.rearLeftMetersPerSecond / maxTranslationSpeed));
        //br_drive.set(1 * (wheelSpeeds.rearRightMetersPerSecond / maxTranslationSpeed));

        //TODO: ADD desired wheel CPS speed current wheel speed CPS to telemetry
        telemetry.addData("X Speed", x_speed);
        telemetry.addData("Y Speed", y_speed);
        telemetry.addData("Rotation Speed", rot_speed);
        telemetry.addData("Front Left Wheel Speed", fl_drive.encoder.getRate());
        telemetry.addData("Front Right Wheel Speed", fr_drive.encoder.getRate());
        telemetry.addData("Back Left Wheel Speed", bl_drive.encoder.getRate());
        telemetry.addData("Back Right Wheel Speed", br_drive.encoder.getRate());
        telemetry.addData("Front Right Encoder Revolutions", fr_drive.encoder.getRevolutions());
        telemetry.addData("Front Left Encoder Revolutions", fl_drive.encoder.getRevolutions());
        telemetry.addData("Back Right Encoder Revolutions", br_drive.encoder.getRevolutions());
        telemetry.addData("Back Left Encoder Revolutions", bl_drive.encoder.getRevolutions());
        //telemetry.addData("Current Pose Y", m_pose.getY());
        //telemetry.addData("Current Pose X", m_pose.getX());
        //telemetry.addData("Current Pose Rotation", m_pose.getRotation().getDegrees());
        telemetry.addData("Distance Y (In meters)", getDistance());
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
                -fl_encoder_current_position_with_offset +
                br_encoder_current_position_with_offset +
                bl_encoder_current_position_with_offset) / 4) * wheelCircumference;
        return -average;
    }

    public void clearEncoderPulse() {
        fr_motor_offset = fr_drive.encoder.getRevolutions();
        fl_motor_offset = fl_drive.encoder.getRevolutions();
        br_motor_offset =  br_drive.encoder.getRevolutions();
        bl_motor_offset = bl_drive.encoder.getRevolutions();
    }
}
