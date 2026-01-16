package frc.robot.subsystems.topdeck;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor;
    private final VelocityVoltage velocityControl;

    public ShooterSubsystem() {
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
        velocityControl = new VelocityVoltage(0);

        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID configuration for velocity control
        config.Slot0.kP = 0.1; // Tune these values for your mechanism
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12; // Feedforward gain

        // Motor configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterMotor.getConfigurator().apply(config);
    }

    /**
     * Spins the shooter to the target velocity
     */
    public void runShooter() {
        shooterMotor.setControl(velocityControl.withVelocity(TARGET_VELOCITY_RPS));
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPS Target velocity in rotations per second
     */
    public void runShooterAtVelocity(double velocityRPS) {
        shooterMotor.setControl(velocityControl.withVelocity(velocityRPS));
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
     * @return Current velocity in rotations per second
     */
    public double getVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Checks if the shooter is at target velocity
     * 
     * @return true if within tolerance
     */
    public boolean atTargetVelocity() {
        double tolerance = 2.0; // RPS tolerance
        return Math.abs(getVelocity() - TARGET_VELOCITY_RPS) < tolerance;
    }

    /**
     * Gets the motor temperature
     * 
     * @return Temperature in Celsius
     */
    public double getTemperature() {
        return shooterMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add SmartDashboard updates here
        // SmartDashboard.putNumber("Shooter Velocity", getVelocity());
        // SmartDashboard.putBoolean("At Target", atTargetVelocity());
    }
}