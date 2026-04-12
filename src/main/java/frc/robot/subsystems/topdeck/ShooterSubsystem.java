package frc.robot.subsystems.topdeck;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.Optional;

public class ShooterSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry targetVelocityEntry = tab
            .add("Target Velocity RPM", TARGET_VELOCITY_RPM)
            .getEntry();
    GenericEntry motor1VelocityEntry = tab
            .add("Motor 1 Velocity", 1)
            .getEntry();
    GenericEntry motor2VelocityEntry = tab
            .add("Motor 2 Velocity", 0)
            .getEntry();
    GenericEntry motor3VelocityEntry = tab
            .add("Motor 3 Velocity", 0)
            .getEntry();
    GenericEntry motor4VelocityEntry = tab
            .add("Motor 4 Velocity", 0)
            .getEntry();

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private SparkFlex shooterMotor = new SparkFlex(SHOOTER_MOTOR_ID_1, MotorType.kBrushless);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.069689, 0.10839, 0.0077901); // kS, kV -

    private SparkFlex shooterMotor2 = new SparkFlex(SHOOTER_MOTOR_ID_2, MotorType.kBrushless);
    private SimpleMotorFeedforward feedforward2 = new SimpleMotorFeedforward(0.087923, 0.10849, 0.0089868); // kS, kV -

    private SparkFlex shooterMotor3 = new SparkFlex(SHOOTER_MOTOR_ID_3, MotorType.kBrushless);
    private SimpleMotorFeedforward feedforward3 = new SimpleMotorFeedforward(0.058578, 0.10867, 0.0085603); // kS, kV -
                                                                                                            // // these!

    private SparkFlex shooterMotor4 = new SparkFlex(SHOOTER_MOTOR_ID_4, MotorType.kBrushless);
    private SimpleMotorFeedforward feedforward4 = new SimpleMotorFeedforward(0.037203, 0.1083, 0.0090113); // kS, kV -
    // private final RelativeEncoder encoder;
    // private final RelativeEncoder encoder2;
    private PIDController pidController;
    private PIDController pidController2;

    // PID Constants - tune these!
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ShooterSubsystem() {
        pidController = new PIDController(kP, kI, kD);
        pidController2 = new PIDController(kP, kI, kD);

        configureMotor();
    }

    private void configureMotor() {

        SparkFlexConfig s1 = new SparkFlexConfig();
        s1.disableFollowerMode();
        s1.smartCurrentLimit(40, 40);
        s1.idleMode(IdleMode.kCoast);
        s1.inverted(true);
        shooterMotor.configure(s1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(s1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor3.configure(s1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor4.configure(s1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Spins the shooter to the target velocity (reads RPM from Shuffleboard
     * slider).
     */
    public boolean runShooter() {
        var velocity = targetVelocityEntry.getDouble(5000);
        return runShooterAtVelocity(velocity);
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    /**
     * Spins the shooter to the interpolated RPM for a given distance to the target.
     * Drop-in replacement for {@link #runShooter()} when distance is known.
     *
     * Uses {@link Constants.ShooterConstants#SHOOTER_RPM_MAP} to linearly
     * interpolate
     * between empirically tested (distance, RPM) pairs. Clamps to the nearest
     * endpoint
     * outside the tested range [12.97, 15.10] ft.
     *
     * @param distanceFeet distance to the target in feet (e.g. from vision or
     *                     odometry)
     * @return true if the shooter is within RPM_TOLERANCE of the interpolated
     *         target
     */
    public boolean runShooterFromDistance(double distanceFeet) {
        double targetRPM = SHOOTER_RPM_MAP.get(distanceFeet);
        SmartDashboard.putNumber("Shooter/Interp Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Distance Ft", distanceFeet);
        return runShooterAtVelocity(targetRPM);
    }

    private void setRPM(Double rpm) {
        shooterMotor.setVoltage(feedforward.calculate(rpm / 60.0)); // Convert RPM to RPS for feedforward calculation
        shooterMotor2.setVoltage(feedforward2.calculate(rpm / 60.0));
        shooterMotor3.setVoltage(feedforward3.calculate(rpm / 60.0));
        shooterMotor4.setVoltage(feedforward4.calculate(rpm / 60.0));
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPM Target velocity in rotations per minute (RPM)
     */
    public boolean runShooterAtVelocity(double velocityRPM) {
        setRPM(velocityRPM);
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
        return Math.abs(getVelocity() - velocityRPM) < RPM_TOLERANCE; // true when within tolerance of target
    }

    /**
     * Runs the shooter at the interpolated RPM for the robot's current field pose.
     * Selects the hub position based on the current alliance (red/blue), computes
     * the Euclidean distance to it, then looks up the optimal RPM from test data.
     *
     * @param robotPose current robot pose from odometry/vision (meters, WPILib
     *                  coords)
     * @return true if the shooter is within RPM_TOLERANCE of the interpolated
     *         target
     */
    public boolean runShooterAtDistance(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance()
                .map(a -> a == Alliance.Red)
                .orElse(true); // default to red if alliance unknown

        double hubX = isRed
                ? Constants.HoodConstants.HUB_X_METERS
                : Constants.HoodConstants.BLUE_HUB_X_METERS;
        double hubY = Constants.HoodConstants.HUB_Y_METERS; // same for both alliances

        double distanceMeters = Math.hypot(hubX - robotPose.getX(), hubY - robotPose.getY());
        double targetRPM = Constants.ShooterConstants.SHOOTER_RPM_MAP.get(distanceMeters);

        SmartDashboard.putString("Shooter/Alliance", isRed ? "Red" : "Blue");
        SmartDashboard.putNumber("Shooter/Distance to Hub (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Interpolated Target RPM", targetRPM);

        return runShooterAtVelocity(targetRPM);
    }

    private void setMotors(double setpoint) {
        System.out.print("Moving");
        shooterMotor.set(setpoint);
        shooterMotor2.set(setpoint);
        shooterMotor3.set(setpoint);
        shooterMotor4.set(setpoint);
    }

    /**
     * Stops the shooter motor
     */
    public void stopShooter() {
        setMotors(0);
    }

    public void spinUpShooter() {
        setMotors(0.5);
    }

    /**
     * Gets the current velocity of the shooter
     * 
     * @return Current velocity in rotations per minute (RPM)
     */
    public double getVelocity() {
        return (shooterMotor.getEncoder().getVelocity() + shooterMotor2.getEncoder().getVelocity()
                + shooterMotor3.getEncoder().getVelocity() + shooterMotor4.getEncoder().getVelocity())
                / Math.max(1, Math.signum(Math.floor(shooterMotor.getEncoder().getVelocity() / 1000.0)) +
                        Math.signum(Math.floor(shooterMotor2.getEncoder().getVelocity() / 1000.0)) +
                        Math.signum(Math.floor(shooterMotor3.getEncoder().getVelocity() / 1000.0)) +
                        Math.signum(Math.floor(shooterMotor4.getEncoder().getVelocity() / 1000.0)));
        // return (shooterMotor.getRotorVelocity().getValueAsDouble());
    }

    public void reverse() {
        setMotors(-0.5);
    }

    // /**
    // * Checks if the shooter is at target velocity
    // *
    // * @return true if within tolerance
    // */
    // public boolean atTargetVelocity() {
    // double tolerance = 100.0; // RPM tolerance
    // return (Math.abs(getVelocity() - TARGET_VELOCITY_RPM) < tolerance);
    // }

    @Override
    public void periodic() {
        // if(isHubActive())
        // shooterMotorFollower.setControl(new Follower(SHOOTER_MOTOR_ID_RIGHT,
        // MotorAlignmentValue.Opposed));

        // SmartDashboard.putNumber("Shooter/Velocity RPM", getVelocity());
        // motor1VelocityEntry.setDouble(encoder.getVelocity());
        // motor2VelocityEntry.setDouble(encoder2.getVelocity());
        SmartDashboard.putNumber("Shooter/Target RPM", TARGET_VELOCITY_RPM);
        // SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        SmartDashboard.putNumber("Motor Position", shooterMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Motor Velocity", shooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Motor VOLTAGE", shooterMotor.getAppliedOutput());

        // FWF - commented this because we're changing the advancer to a neo
        // motor1VelocityEntry.

    }
}