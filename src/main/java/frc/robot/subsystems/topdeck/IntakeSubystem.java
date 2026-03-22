package frc.robot.subsystems.topdeck;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubystem extends SubsystemBase {

    private TalonFX intakeMotor;
    private SparkMax rotationMotor;

    public IntakeSubystem() {
        intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
        rotationMotor = new SparkMax(INTAKE_SLIDER1_ID, MotorType.kBrushless);
        configMotors();
    }

    private void configMotors() {
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        SparkBaseConfig RotatorConfig = new SparkMaxConfig();

        intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeMotor.getConfigurator().apply(intakeMotorConfig);

        RotatorConfig.smartCurrentLimit(30, 30);
        RotatorConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        RotatorConfig.inverted(true);
        RotatorConfig.disableFollowerMode();
        // RotatorConfig.softLimit.forwardSoftLimitEnabled(true);
        // RotatorConfig.softLimit.forwardSoftLimit(Intake_Up_SetPoint);
        // RotatorConfig.softLimit.reverseSoftLimitEnabled(true);
        // RotatorConfig.softLimit.reverseSoftLimit(Intake_Down_SetPoint);
        rotationMotor.getEncoder().setPosition(0);
        rotationMotor.configure(RotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean intakeMoving() {
        return rotationMotor.get() != 0;
    }

    public void Outake() {
        intakeMotor.set(-INTAKE_SPEED);
    }

    public void Intake() {
        Constants.States.INTAKE_ON = true;
        intakeMotor.set(INTAKE_SPEED);
    }

    public void stopIntake() {
        Constants.States.INTAKE_ON = false;
        intakeMotor.set(0);
    }

    public void stop() {
        rotationMotor.stopMotor();
    }

    private boolean IMoveDownToIntakeI() {
        rotationMotor.set(-0.5);

        return rotationMotor.getEncoder().getPosition() <= Intake_Down_SetPoint;
    }

    private boolean IMoveUpToStoreI() {
        rotationMotor.set(0.5);

        return rotationMotor.getEncoder().getPosition() >= Intake_Up_SetPoint;
    }

    public boolean MoveUpToStore() {
        if (IMoveDownToIntakeI()) {
            stop();
            Constants.States.INTAKE_IN = true;
            return true;
        }
        return false;
    }

    public boolean MoveDownToIntake() {
        if (IMoveUpToStoreI()) {
            stop();
            Constants.States.INTAKE_IN = false;
            return true;
        }
        return false;
    }

    public void testSliders(double JoystickValue) {
        rotationMotor.set(JoystickValue);
    }

    @Override
    public void periodic() {
        if (Constants.States.CLIMBER_DOWN) {
            MoveUpToStore();
        }
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Motor Position ", rotationMotor.getEncoder().getPosition());
    }
}
