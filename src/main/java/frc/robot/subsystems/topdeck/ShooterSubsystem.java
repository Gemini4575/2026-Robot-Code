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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry maxspeedEntry = tab
            .add("Max Speed Shooter", 1)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .getEntry();
    GenericEntry Vortex1Entry = tab
            .add("Target Velocity Shooter", 0)
            .getEntry();
    GenericEntry Vortex2Entry = tab
            .add("Target Velocity Shooter 2", 0)
            .getEntry();

    private final SparkMax shooterMotor;
    private final SparkMax shooterMotor2;
    private final RelativeEncoder encoder;
    private final RelativeEncoder encoder2;
    private PIDController pidController;
    private PIDController pidController2;

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(SHOOTER_MOTOR_ID_RIGHT, MotorType.kBrushless);
        shooterMotor2 = new SparkMax(SHOOTER_MOTOR_ID_LEFT, MotorType.kBrushless);
        encoder = shooterMotor.getEncoder();
        encoder2 = shooterMotor2.getEncoder();

        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        SparkMaxConfig config2 = new SparkMaxConfig();

        config.disableFollowerMode();
        config2.disableFollowerMode();

        config2.inverted(true);
        config.inverted(false);

        pidController = new PIDController(0.00005, 0, 0);
        pidController2 = new PIDController(0.00005, 0, 0);

        pidController.setIZone(0.06);
        pidController2.setIZone(0.06);

        // Motor configuration
        config.idleMode(IdleMode.kCoast);
        config2.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(40); // Current limit in amps
        config2.smartCurrentLimit(40);


        // Optional: Set voltage compensation
        config.voltageCompensation(12.0);
        config2.voltageCompensation(12.0);

        // Apply configuration
        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Spins the shooter to the target velocity
     */
    public void runShooter() {
        // pidController.calculate(TARGET_VELOCITY_RPS, ControlType.kVelocity);
        // pidController2.calculate(TARGET_VELOCITY_RPS, ControlType.kVelocity);
        shooterMotor.set(maxspeedEntry.getDouble(1.0));
        shooterMotor2.set(maxspeedEntry.getDouble(1.0));
    }

      private double getCurrentSpeedAsPercentage1() {
        return encoder.getVelocity() / MOTOR_MAX_RPM;
    }

      private double getCurrentSpeedAsPercentage2() {
        return encoder2.getVelocity() / MOTOR_MAX_RPM;
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPM Target velocity in rotations per minute (RPM)
     */
    public void runShooterAtVelocity(double velocityRPM) {
        // shooterMotor.set(getCurrentSpeedAsPercentage1() + pidController.calculate(getCurrentSpeedAsPercentage1(), velocityRPM));
        // shooterMotor2.set(getCurrentSpeedAsPercentage2() + pidController2.calculate(getCurrentSpeedAsPercentage2(), velocityRPM));
            shooterMotor.set(maxspeedEntry.getDouble(1.0));
            shooterMotor2.set(maxspeedEntry.getDouble(1.0));
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
        return encoder.getVelocity() + encoder2.getVelocity() / 2.0; // Average velocity of both motors
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
        // This method will be called once per scheduler run
        // Add SmartDashboard updates here for debugging
        
         SmartDashboard.putNumber("Shooter/Velocity RPM", getVelocity());
        Vortex1Entry.setDouble(shooterMotor.get());
        Vortex2Entry.setDouble(shooterMotor2.get());
        
        //  SmartDashboard.putNumber("Shooter/Target RPM", TARGET_VELOCITY_RPM);
        //  SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        //  SmartDashboard.putNumber("Shooter/Temp C", getTemperature());
        //  SmartDashboard.putNumber("Shooter/Current A", getCurrent());
        //  SmartDashboard.putBoolean("Shooter/Healthy", isMotorHealthy());
         
    }
}