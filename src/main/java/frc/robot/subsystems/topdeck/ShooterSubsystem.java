package frc.robot.subsystems.topDeck;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.networktables.GenericEntry;
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

    private final SparkMax shooterMotor1;
    private final SparkMax shooterMotor2;
    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;
    private final SparkClosedLoopController pidController1;
    private final SparkClosedLoopController pidController2;

    public ShooterSubsystem() {
        shooterMotor1 = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor2 = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        encoder1 = shooterMotor1.getEncoder();
        encoder2 = shooterMotor2.getEncoder();
        pidController1 = shooterMotor1.getClosedLoopController();
        pidController2 = shooterMotor2.getClosedLoopController();

        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config1 = new SparkMaxConfig();
        SparkMaxConfig config2 = new SparkMaxConfig();

        config1.disableFollowerMode();
        config2.disableFollowerMode();

        // PID configuration for velocity control (Slot 0)
        config1.closedLoop.pid(0.0001, 0.0, 0.0, ClosedLoopSlot.kSlot0); // P, I, D
        config2.closedLoop.pid(0.0001, 0.0, 0.0);
        config1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // Motor configuration
        config1.idleMode(IdleMode.kCoast);
        config2.idleMode(IdleMode.kCoast);
        config1.smartCurrentLimit(40); // Current limit in amps
        config2.smartCurrentLimit(40);

        // Optional: Set voltage compensation
        config1.voltageCompensation(12.0);
        config2.voltageCompensation(12.0);

        // Apply configuration
        shooterMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Spins the shooter to the target velocity
     */
    public void runShooter() {
        pidController1.setSetpoint(TARGET_VELOCITY_RPS, ControlType.kVelocity);
        pidController2.setSetpoint(TARGET_VELOCITY_RPS, ControlType.kVelocity);
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPM Target velocity in rotations per minute (RPM)
     */
    public void runShooterAtVelocity(double velocityRPM) {
        pidController1.setSetpoint(velocityRPM, ControlType.kVelocity);
        pidController2.setSetpoint(velocityRPM, ControlType.kVelocity);
    }

    /**
     * Stops the shooter motor
     */
    public void stopShooter() {
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    /**
     * Gets the current velocity of the shooter
     * 
     * @return Current velocity in rotations per minute (RPM)
     */
    public double getVelocity1() {
        return encoder1.getVelocity();
    }

    public double getVelocity2() {
        return encoder2.getVelocity();
    }

    /**
     * Checks if the shooter is at target velocity
     * 
     * @return true if within tolerance
     */
    public boolean atTargetVelocity() {
        double tolerance = 100.0; // RPM tolerance
        return (Math.abs(getVelocity1() - TARGET_VELOCITY_RPS) < tolerance)
                && (Math.abs(getVelocity2() - TARGET_VELOCITY_RPS) < tolerance);
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