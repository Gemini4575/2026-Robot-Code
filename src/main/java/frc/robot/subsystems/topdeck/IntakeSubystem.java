package frc.robot.subsystems.topdeck;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubystem extends SubsystemBase {

    private SparkMax intakeMotor;
    private SparkMax slider1;
    private SparkMax slider2;

    public IntakeSubystem() {
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        slider1 = new SparkMax(INTAKE_SLIDER1_ID, MotorType.kBrushed);
        slider2 = new SparkMax(INTAKE_SLIDER2_ID, MotorType.kBrushed);
        configMotors();
    }

    private void configMotors() {
        SparkBaseConfig intakeMotorConfig = new SparkMaxConfig();
        SparkBaseConfig sliderConfig = new SparkMaxConfig();
        intakeMotorConfig.smartCurrentLimit(30, 30);
        intakeMotorConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        intakeMotorConfig.inverted(false);
        intakeMotorConfig.disableFollowerMode();
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sliderConfig.smartCurrentLimit(30, 30);
        sliderConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        sliderConfig.inverted(false);
        sliderConfig.disableFollowerMode();
        slider1.configure(sliderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sliderConfig.inverted(true);
        slider2.configure(sliderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void Outake() {
        intakeMotor.set(INTAKE_SPEED);
    }

    public void Intake() {
        intakeMotor.set(-INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void extendIntake() {
        slider1.set(0.5);
        slider2.set(0.5);
    }

    public void retractIntake() {
        slider1.set(-0.5);
        slider2.set(-0.5);
    }

    public void stopSliders() {
        slider1.set(0);
        slider2.set(0);
    }
    
    public void testSliders(double JoystickValue) {
        slider1.set(JoystickValue);
        slider2.set(JoystickValue);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getEncoder().getVelocity());
    }
}
