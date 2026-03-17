package frc.robot.subsystems.topdeck;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry targetVelocityEntry = tab
            .add("Target Velocity RPM", TARGET_VELOCITY_RPM)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
    GenericEntry motor1VelocityEntry = tab
            .add("Motor 1 Velocity", 1)
            .getEntry();
    GenericEntry motor2VelocityEntry = tab
            .add("Motor 2 Velocity", 0)
            .getEntry();

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private SparkMax shooterMotor = new SparkMax(SHOOTER_MOTOR_ID_1, MotorType.kBrushless);
    private SparkMax shooterMotor2 = new SparkMax(SHOOTER_MOTOR_ID_2, MotorType.kBrushless);
    private SparkMax shooterMotor3 = new SparkMax(SHOOTER_MOTOR_ID_3, MotorType.kBrushless);
    private SparkMax shooterMotor4 = new SparkMax(SHOOTER_MOTOR_ID_4, MotorType.kBrushless);
    // private final RelativeEncoder encoder;
    // private final RelativeEncoder encoder2;
    private PIDController pidController;
    private PIDController pidController2;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Default ramp rate (1 V/s)
                    edu.wpi.first.units.Units.Volts.of(4), // Limit to 4V for safety
                    null, // Default timeout
                    null // Default state logger
            ),
            new SysIdRoutine.Mechanism(
                    (voltage) -> {
                        // Drive all 4 motors
                        shooterMotor.setVoltage(voltage);
                        shooterMotor2.setVoltage(voltage);
                        shooterMotor3.setVoltage(voltage);
                        shooterMotor4.setVoltage(voltage);
                    },
                    (log) -> {
                        // Log all 4 motors separately so you get results for each
                        log.motor("shooter-1")
                                .voltage(edu.wpi.first.units.Units.Volts.of(
                                        shooterMotor.getBusVoltage() * shooterMotor.getAppliedOutput()))
                                .angularPosition(edu.wpi.first.units.Units.Rotations.of(
                                        shooterMotor.getEncoder().getPosition()))
                                .angularVelocity(edu.wpi.first.units.Units.RotationsPerSecond.of(
                                        shooterMotor.getEncoder().getVelocity() / 60.0));
                        log.motor("shooter-2")
                                .voltage(edu.wpi.first.units.Units.Volts.of(
                                        shooterMotor2.getBusVoltage() * shooterMotor2.getAppliedOutput()))
                                .angularPosition(edu.wpi.first.units.Units.Rotations.of(
                                        shooterMotor2.getEncoder().getPosition()))
                                .angularVelocity(edu.wpi.first.units.Units.RotationsPerSecond.of(
                                        shooterMotor2.getEncoder().getVelocity() / 60.0));
                        log.motor("shooter-3")
                                .voltage(edu.wpi.first.units.Units.Volts.of(
                                        shooterMotor3.getBusVoltage() * shooterMotor3.getAppliedOutput()))
                                .angularPosition(edu.wpi.first.units.Units.Rotations.of(
                                        shooterMotor3.getEncoder().getPosition()))
                                .angularVelocity(edu.wpi.first.units.Units.RotationsPerSecond.of(
                                        shooterMotor3.getEncoder().getVelocity() / 60.0));
                        log.motor("shooter-4")
                                .voltage(edu.wpi.first.units.Units.Volts.of(
                                        shooterMotor4.getBusVoltage() * shooterMotor4.getAppliedOutput()))
                                .angularPosition(edu.wpi.first.units.Units.Rotations.of(
                                        shooterMotor4.getEncoder().getPosition()))
                                .angularVelocity(edu.wpi.first.units.Units.RotationsPerSecond.of(
                                        shooterMotor4.getEncoder().getVelocity() / 60.0));
                    },
                    this));

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
        SparkMaxConfig s1 = new SparkMaxConfig();
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
     * Spins the shooter to the target velocity
     */
    public void runShooter() {
        runShooterAtVelocity(targetVelocityEntry.getDouble(1));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    private void SysidTesting(Voltage voltage) {
        shooterMotor.setVoltage(voltage);
        shooterMotor2.setVoltage(voltage);
        shooterMotor3.setVoltage(voltage);
        shooterMotor4.setVoltage(voltage);
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPM Target velocity in rotations per minute (RPM)
     */
    public void runShooterAtVelocity(double velocityRPM) {
        setMotors(velocityRPM);
        System.out.print("Shooting");
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
                + shooterMotor3.getEncoder().getVelocity() + shooterMotor4.getEncoder().getVelocity()) / 4.0;
        // return (shooterMotor.getRotorVelocity().getValueAsDouble());
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

    }
}