package frc.robot.subsystems.topDeck;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry maxspeedEntry = tab
            .add("Max Speed Shooter", 1)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .getEntry();

    private final SparkMax shooterMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        encoder = shooterMotor.getEncoder();
        pidController = shooterMotor.getClosedLoopController();

        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        // PID configuration for velocity control (Slot 0)
        config.closedLoop.pid(0.0001, 0.0, 0.0, ClosedLoopSlot.kSlot0); // P, I, D, FF
        config.closedLoop.feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder);

        // Motor configuration
        config.idleMode(SparkMaxConfig.IdleMode.kCoast);
        config.smartCurrentLimit(40); // Current limit in amps

        // Optional: Set voltage compensation
        config.voltageCompensation(12.0);

        // Apply configuration
        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Spins the shooter to the target velocity
     */
    public void runShooter() {
        pidController.setSetpoint(TARGET_VELOCITY_RPS, ControlType.kVelocity);
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPM Target velocity in rotations per minute (RPM)
     */
    public void runShooterAtVelocity(double velocityRPM) {
        pidController.setSetpoint(velocityRPM, ControlType.kVelocity);
    }

    /**
     * Stops the shooter motor
     */
    public void stopShooter() {
        shooterMotor.stopMotor();
    }

    /**
     * Gets the current velocity of the shooter
     * 
     * @return Current velocity in rotations per minute (RPM)
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Checks if the shooter is at target velocity
     * 
     * @return true if within tolerance
     */
    public boolean atTargetVelocity() {
        double tolerance = 100.0; // RPM tolerance
        return Math.abs(getVelocity() - TARGET_VELOCITY_RPS) < tolerance;
    }

    /**
     * Gets the motor temperature
     * 
     * @return Temperature in Celsius
     */
    public double getTemperature() {
        return shooterMotor.getMotorTemperature();
    }

    /**
     * Gets the motor current draw
     * 
     * @return Current in Amps
     */
    public double getCurrent() {
        return shooterMotor.getOutputCurrent();
    }

    /**
     * Checks if motor is operating within safe parameters
     * 
     * @return true if motor is healthy
     */
    public boolean isMotorHealthy() {
        return getTemperature() < 80.0 && getCurrent() < 35.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add SmartDashboard updates here for debugging
        /*
         * SmartDashboard.putNumber("Shooter/Velocity RPM", getVelocity());
         * SmartDashboard.putNumber("Shooter/Target RPM", TARGET_VELOCITY_RPM);
         * SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
         * SmartDashboard.putNumber("Shooter/Temp C", getTemperature());
         * SmartDashboard.putNumber("Shooter/Current A", getCurrent());
         * SmartDashboard.putBoolean("Shooter/Healthy", isMotorHealthy());
         */
    }
}