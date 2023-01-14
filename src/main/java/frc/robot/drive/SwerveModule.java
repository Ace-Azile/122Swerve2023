package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.motors.NKTalonFX;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private NKTalonFX drive;
    private NKTalonFX turn;
    private CANCoder turnEncoder;
    private Rotation2d angleOffset = new Rotation2d();
    private SimpleMotorFeedforward feedforward;

    private int id;

    private double lastAngle;
    // initialize all the things
    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, Rotation2d angleOffset) {
        id = (driveMotorID / 10);
        initEncoder(encoderID);
        initDriveMotor(driveMotorID);
        initTurnMotor(turnMotorID);
        // what does feed forward do? real life correcter
        this.feedforward = new SimpleMotorFeedforward(ModuleConstants.kDriveS, ModuleConstants.kDriveV,
                ModuleConstants.kDriveA);
        this.angleOffset = angleOffset;
    }
    // get current velocity and angle of swerve module
    public SwerveModuleState getCurrentState() {
        double velocity = getVelocityMPS();
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModuleState(velocity, angle);
    }
    // get position of swerve module
    public SwerveModulePosition getPosition() {
        double distance = getDistanceMeters(); // if this always returns 0,0; then will all distances be 0
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModulePosition(distance, angle);
    }
    // change module to desired velocity and angle
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        Rotation2d currentAngleRotation2d = getAngleRotation2d();
        // where is SwerveModuleState.optimize defined?
        desiredState = SwerveModuleState.optimize(desiredState, currentAngleRotation2d);
        // set speed to desired speed
        if (isOpenLoop) {
            // if true then change speed
            double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kMaxSpeed;
            drive.set(ControlMode.PercentOutput, percentOutput);
        } else {
            // else feedforward? what is feedforward? does it mean continue at current speed
            // sets voltage to continue at the current speed
            drive.setVoltage(feedforward.calculate(desiredState.speedMetersPerSecond)
                    + ModuleConstants.kDriveP * (desiredState.speedMetersPerSecond - getVelocityMPS()));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (ModuleConstants.kMaxSpeed * 0.01)) ? lastAngle
                : desiredState.angle.getDegrees();
        // if desired angle is less than 1 degree different than the current angle than move the motor
        if (Math.abs(currentAngleRotation2d.minus(desiredState.angle).getDegrees()) > 1) {
            turn.set(ControlMode.Position, Conversions.degreesToFalcon(angle, ModuleConstants.kTurnGearRatio));
        } else {
            // if not do not turn the wheel
            turn.set(ControlMode.PercentOutput, 0);
        }
        // set current angle to the previous angle
        lastAngle = angle;
    }
    // move all wheels back to 0
    private void resetToAbsolute() {
        lastAngle = turnEncoder.getAbsolutePosition() - angleOffset.getDegrees();
        double absolutePosition = Conversions.falconToDegrees(lastAngle, ModuleConstants.kDriveGearRatio);
        turn.setSelectedSensorPosition(absolutePosition);
    }
    // initialize the drive motor
    private void initDriveMotor(int driveMotorID) {
        drive = new NKTalonFX(driveMotorID);

        drive.configFactoryDefault();
        drive.configAllSettings(ModuleConstants.kDriveMotorConfig);
        drive.setInverted(ModuleConstants.kDriveMotorInverted);
        drive.setNeutralMode(ModuleConstants.kDriveMotorNeutral);
        drive.setSelectedSensorPosition(0);

    }
    // initialize the turn motor
    private void initTurnMotor(int turnMotorID) {
        turn = new NKTalonFX(turnMotorID);
        turn.configFactoryDefault();
        turn.configAllSettings(ModuleConstants.kTurnMotorConfig);
        turn.setInverted(ModuleConstants.kTurnMotorInverted);
        // turn.setSensorPhase(true);
        turn.setNeutralMode(ModuleConstants.kTurnMotorNeutral);
        turn.configRemoteFeedbackFilter(turnEncoder, 0);
        turn.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        resetToAbsolute();
    }
    // initialize the encoder
    private void initEncoder(int encoderID) {
        turnEncoder = new CANCoder(encoderID);

        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(ModuleConstants.kEncoderConfig);
    }
    // update smartdash with current data
    public void updateSmartDash() {
        SmartDashboard.putNumber(id + " Module Encoder Raw Position", turnEncoder.getPosition() % 360);
        SmartDashboard.putNumber(id + " Motor Integrated Sensor Position", turn.getSelectedSensorPosition());
        SmartDashboard.putNumber(id + " Module Angle", getAngleRotation2d().getDegrees());
    }
    // get the angle of the wheel
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(turn.getSelectedSensorPosition(), ModuleConstants.kTurnGearRatio)
                        - angleOffset.getDegrees());
    }
    //ditance is 0
    public double getDistanceMeters() {
        return 0.0;
    }
    // get velocity
    public double getVelocityMPS() {
        return Conversions.falconToMPS(drive.getSelectedSensorVelocity(), ModuleConstants.kWheelCircumference,
                ModuleConstants.kDriveGearRatio);
    }
    //test the drive motor
    public void testDriveSpinny(double output) {
        drive.set(output);
    }
    // test the turn motor
    public void testTurnSpinny(double output) {
        turn.set(output);
    }
    // stop both motors
    public void testStopSpinny() {
        drive.set(0);
        turn.set(0);
    }
    // set voltage of the drive motor
    public void setDriveVoltage(double voltage) {
        drive.setVoltage(voltage);
    }
    // conversion ratios of falcon tics to degrees and vice versa and meters per second to
    // units per 100ms
    public static final class Conversions {
        public static double falconToDegrees(double counts, double gearRatio) {
            // ratio = motor/wheel
            return counts * (360.0 / (gearRatio * 2048.0));
        }

        public static double degreesToFalcon(double degrees, double gearRatio) {
            double ticks = degrees / (360.0 / (gearRatio * 2048.0));
            return ticks;
        }

        /**
         * Converts a falcon motor position into distance traveled
         * 
         * @param falconPosition falcon position sensor counts
         * @param circumference  wheel circumference in meters
         * @param gearRatio      motor rotations/wheel rotations
         * @return distance traveled in meters
         */
        public static double falconToMeters(double falconPosition, double circumference, double gearRatio) {
            double motorRotations = falconPosition / 2048.0;
            // rotations * m * g, where g is in units motor/wheel rotation
            double distance = motorRotations * circumference / gearRatio;
            return distance;
        }

        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * (600.0 / 2048.0);
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            return sensorCounts;
        }

        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / 60;
            return wheelMPS;
        }

        public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
            double wheelRPM = ((velocity * 60) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }
    }
}
