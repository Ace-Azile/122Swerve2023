package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.sds.SDSModuleType;

public class Constants {
    public static final int kDriverPort = 0;
    public static final SPI.Port kNavXPort = SPI.Port.kMXP;

    public static final SDSModuleType kSDSModule = SDSModuleType.MK4I_L2;

    public static final double kTrackwidthMeters = 0.4;
    public static final double kWheelbaseMeters = 0.4;

    public static final double kDefaultAxisDeadband = 0.15;

    // rpm / (60 s / min) * (g * C m / rot) where g is the gear ratio and C is the
    // curcumference of the wheel
    /**
     * Maximum translational velocity of the robot in meters per second
     */
    public static final double kMaxTranslationalVelocity = (6380.0 / 60) * kSDSModule.getWheelDiameter() * Math.PI
            * kSDSModule.getDriveReduction();
    /**
     * Maximum rotational velocity of the robot
     */
    public static final double kMaxRotationalVelocity = kMaxTranslationalVelocity
            / Math.hypot(kTrackwidthMeters / 2, kWheelbaseMeters / 2);
    public static final boolean kIsFieldRelative = false;

    public static final int kFrontLeftDriveMotorID = 10;
    public static final int kFrontLeftTurnMotorID = 11;
    public static final int kFrontLeftEncoderID = 12;
    public static final Rotation2d kFrontLeftOffset = Rotation2d.fromDegrees(283.447).unaryMinus(); // -76.201
    public static final Translation2d kFrontLeftPosition = new Translation2d(kTrackwidthMeters / 2.0,
            kWheelbaseMeters / 2.0);

    public static final int kFrontRightDriveMotorID = 20;
    public static final int kFrontRightTurnMotorID = 21;
    public static final int kFrontRightEncoderID = 22;
    public static final Rotation2d kFrontRightOffset = Rotation2d.fromDegrees(139.658); // 139.658
    public static final Translation2d kFrontRightPosition = new Translation2d(kTrackwidthMeters / 2.0,
            -kWheelbaseMeters / 2.0);

    public static final int kBackLeftDriveMotorID = 30;
    public static final int kBackLeftTurnMotorID = 31;
    public static final int kBackLeftEncoderID = 32;
    public static final Rotation2d kBackLeftOffset = Rotation2d.fromDegrees(95.977); // 96.680
    public static final Translation2d kBackLeftPosition = new Translation2d(-kTrackwidthMeters / 2.0,
            kWheelbaseMeters / 2.0);

    public static final int kBackRightDriveMotorID = 40;
    public static final int kBackRightTurnMotorID = 41;
    public static final int kBackRightEncoderID = 42;
    public static final Rotation2d kBackRightOffset = Rotation2d.fromDegrees(265.517); // -93.867
    public static final Translation2d kBackRightPosition = new Translation2d(-kTrackwidthMeters / 2.0,
            -kWheelbaseMeters / 2.0);

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(kFrontLeftPosition,
            kFrontRightPosition, kBackLeftPosition, kBackRightPosition);

    public static final class ModuleConstants {

        public static final double kMaxSpeed = frc.robot.Constants.kMaxTranslationalVelocity;
        public static final double kWheelDiameterMeters = kSDSModule.getWheelDiameter();
        public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
        // ratio is motor rot / wheel rot
        public static final double kDriveGearRatio = 1 / kSDSModule.getDriveReduction();
        public static final double kTurnGearRatio = 1 / kSDSModule.getSteerReduction();

        public static final boolean kDriveMotorInverted = kSDSModule.isDriveInverted();
        public static final NeutralMode kDriveMotorNeutral = NeutralMode.Coast;
        public static final double kDriveVoltageComp = 12;
        public static final boolean kTurnMotorInverted = true;
        public static final NeutralMode kTurnMotorNeutral = NeutralMode.Brake; // set back to brake to be amazing
        public static final double kTurnVoltageComp = 12;
        // later
        public static final boolean kEncoderInverted = false;

        public static final TalonFXConfiguration kDriveMotorConfig = new TalonFXConfiguration() {
            {
                this.slot0.kP = kDriveP;
                this.slot0.kI = kDriveI;
                this.slot0.kD = kDriveD;
                this.slot0.kF = kDriveF;
                this.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                        kDriveEnableCurrentLimit,
                        kDriveContinuousCurrentLimit,
                        kDrivePeakCurrentLimit,
                        kDrivePeakCurrentDuration);

                this.initializationStrategy = SensorInitializationStrategy.BootToZero;
                this.openloopRamp = kOpenLoopRamp;
                this.closedloopRamp = kClosedLoopRamp;
                this.voltageCompSaturation = kDriveVoltageComp;
                this.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;

            }
        };
        public static final TalonFXConfiguration kTurnMotorConfig = new TalonFXConfiguration() {
            {
                this.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
                this.slot0.kP = kTurnP;
                this.slot0.kI = kTurnI;
                this.slot0.kD = kTurnD;
                this.slot0.kF = kTurnF;
                this.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                        kTurnEnableCurrentLimit,
                        kTurnContinuousCurrentLimit,
                        kTurnPeakCurrentLimit,
                        kTurnPeakCurrentDuration);

                this.initializationStrategy = SensorInitializationStrategy.BootToZero;
                this.voltageCompSaturation = kTurnVoltageComp;
            }
        };
        public static final CANCoderConfiguration kEncoderConfig = new CANCoderConfiguration() {
            {
                this.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                this.sensorDirection = kEncoderInverted;
                this.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                this.sensorTimeBase = SensorTimeBase.PerSecond;
            }
        };

        public static final double kOpenLoopRamp = 0.25;
        public static final double kClosedLoopRamp = 0.0;

        public static final boolean kTurnEnableCurrentLimit = true;
        public static final int kTurnContinuousCurrentLimit = 25;
        public static final int kTurnPeakCurrentLimit = 40;
        public static final double kTurnPeakCurrentDuration = 0.1;

        public static final boolean kDriveEnableCurrentLimit = true;
        public static final int kDriveContinuousCurrentLimit = 35;
        public static final int kDrivePeakCurrentLimit = 60;
        public static final double kDrivePeakCurrentDuration = 0.1;

        public static final double kDriveS = 0.519;
        public static final double kDriveV = 2.306;
        public static final double kDriveA = 0.0;

        public static final double kDriveP = 1.5;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveF = 0; // 0.25 / 0.54 * 0.1;

        public static final double kTurnP = 0.6;
        public static final double kTurnI = 0;
        public static final double kTurnD = 12.0;
        public static final double kTurnF = 0.0;
    }

    public static void main(String[] args) {
        SDSModuleType module = SDSModuleType.MK4_L3;

        double trackwidth = 0.4;
        double wheelbase = 0.4;

        double maxTransVel = (6380.0 / 60) * module.getWheelDiameter() * Math.PI
                * module.getDriveReduction();
        double maxRotVel = kMaxTranslationalVelocity
                / Math.hypot(trackwidth / 2, wheelbase / 2);
        System.out.println("max trans vel: " + maxTransVel + " m/s");
        System.out.println("max rot vel: " + maxRotVel + " rad/s");
        System.out.println("max rot vel: " + maxRotVel * 180 / Math.PI + " deg/s");
    }
}
