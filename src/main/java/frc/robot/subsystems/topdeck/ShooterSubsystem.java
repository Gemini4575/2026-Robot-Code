package frc.robot.subsystems.topdeck;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry targetVelocityEntry = tab
            .add("Target Velocity RPM", TARGET_VELOCITY_RPM)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
    GenericEntry motor1VelocityEntry = tab
            .add("Motor 1 Velocity", 0)
            .getEntry();
    GenericEntry motor2VelocityEntry = tab
            .add("Motor 2 Velocity", 0)
            .getEntry();

    private final SparkMax shooterMotor;
    private final SparkMax shooterMotor2;
    private final RelativeEncoder encoder;
    private final RelativeEncoder encoder2;
    private PIDController pidController;
    private PIDController pidController2;

    // PID Constants - tune these!
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(SHOOTER_MOTOR_ID_RIGHT, MotorType.kBrushless);
        shooterMotor2 = new SparkMax(SHOOTER_MOTOR_ID_LEFT, MotorType.kBrushless);
        encoder = shooterMotor.getEncoder();
        encoder2 = shooterMotor2.getEncoder();

        pidController = new PIDController(kP, kI, kD);
        pidController2 = new PIDController(kP, kI, kD);

        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        SparkMaxConfig config2 = new SparkMaxConfig();

        config.disableFollowerMode();
        config2.disableFollowerMode();

        config2.inverted(true);
        config.inverted(false);

        config.idleMode(IdleMode.kCoast);
        config2.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(40);
        config2.smartCurrentLimit(40);

        config.voltageCompensation(12.0);
        config2.voltageCompensation(12.0);

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Spins the shooter to the target velocity
     */
    public void runShooter() {
        runShooterAtVelocity(TARGET_VELOCITY_RPM);
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPM Target velocity in rotations per minute (RPM)
     */
    public void runShooterAtVelocity(double velocityRPM) {
        // Calculate PID output for each motor
        double output1 = pidController.calculate(encoder.getVelocity(), velocityRPM);
        double output2 = pidController2.calculate(encoder2.getVelocity(), velocityRPM);
        
        // Set motor outputs
        shooterMotor.set(output1);
        shooterMotor2.set(output2);
    }

    /**
     * Stops the shooter motor
     */
    public void stopShooter() {
        shooterMotor.stopMotor();
        shooterMotor2.stopMotor();
    }

    /**
     * Gets the current velocity of the shooter
     * 
     * @return Current velocity in rotations per minute (RPM)
     */
    public double getVelocity() {
        return (encoder.getVelocity() + encoder2.getVelocity()) / 2.0;
    }

    /**
     * Checks if the shooter is at target velocity
     * 
     * @return true if within tolerance
     */
    public boolean atTargetVelocity() {
        double tolerance = 100.0; // RPM tolerance
        return (Math.abs(getVelocity() - TARGET_VELOCITY_RPM) < tolerance);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Velocity RPM", getVelocity());
        motor1VelocityEntry.setDouble(encoder.getVelocity());
        motor2VelocityEntry.setDouble(encoder2.getVelocity());
        SmartDashboard.putNumber("Shooter/Target RPM", TARGET_VELOCITY_RPM);
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
    }
}