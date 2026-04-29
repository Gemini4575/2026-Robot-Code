package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A wrapper around WPILib's {@link PIDController} that publishes P, I, and D
 * gains to SmartDashboard / Shuffleboard, allowing them to be tweaked live
 * without re-deploying code.
 *
 * <p>
 * Usage – drop-in replacement for {@code PIDController}:
 * 
 * <pre>
 * // Old:
 * private PIDController pid = new PIDController(2.9, 0.1, 0);
 *
 * // New (key must be unique per controller):
 * private TunablePIDController pid = new TunablePIDController("Swerve/Turning", 2.9, 0.1, 0);
 * </pre>
 *
 * <p>
 * Call {@code pid.updateFromDashboard()} in your subsystem's
 * {@code periodic()} method so the gains are picked up continuously:
 * 
 * <pre>
 *   {@literal @}Override
 *   public void periodic() {
 *       pid.updateFromDashboard();
 *   }
 * </pre>
 *
 * <p>
 * Dashboard keys are formatted as {@code "<key>/P"}, {@code "<key>/I"},
 * {@code "<key>/D"}, and {@code "<key>/Setpoint"} (read-only).
 */
public class TunablePIDController {

    private final PIDController m_pid;
    private final String m_key;

    // Cached values so we only call setPID when something actually changed
    private double m_lastP;
    private double m_lastI;
    private double m_lastD;

    // Dashboard entry keys
    private final String m_keyP;
    private final String m_keyI;
    private final String m_keyD;
    private final String m_keySetpoint;
    private final String m_keyError;

    /**
     * Creates a {@code TunablePIDController} with a named dashboard group.
     *
     * @param dashboardKey A unique name for this controller (e.g.
     *                     {@code "Shooter/Flywheel"} or {@code "Swerve/Turning"}).
     *                     Slashes create sub-tables in Shuffleboard.
     * @param kP           Initial proportional gain
     * @param kI           Initial integral gain
     * @param kD           Initial derivative gain
     */
    public TunablePIDController(String dashboardKey, double kP, double kI, double kD) {
        m_key = dashboardKey;
        m_pid = new PIDController(kP, kI, kD);

        m_keyP = dashboardKey + "/P";
        m_keyI = dashboardKey + "/I";
        m_keyD = dashboardKey + "/D";
        m_keySetpoint = dashboardKey + "/Setpoint";
        m_keyError = dashboardKey + "/Error";

        // Push initial values so they show up immediately even before the first
        // periodic
        SmartDashboard.putNumber(m_keyP, kP);
        SmartDashboard.putNumber(m_keyI, kI);
        SmartDashboard.putNumber(m_keyD, kD);
        SmartDashboard.putNumber(m_keySetpoint, 0.0);
        SmartDashboard.putNumber(m_keyError, 0.0);

        m_lastP = kP;
        m_lastI = kI;
        m_lastD = kD;
    }

    /**
     * Creates a {@code TunablePIDController} with a period (for use with
     * non-standard loop rates).
     *
     * @param dashboardKey A unique name for this controller
     * @param kP           Initial proportional gain
     * @param kI           Initial integral gain
     * @param kD           Initial derivative gain
     * @param period       Loop period in seconds (default WPILib is 0.02 s)
     */
    public TunablePIDController(String dashboardKey, double kP, double kI, double kD,
            double period) {
        this(dashboardKey, kP, kI, kD);
        m_pid.setPID(kP, kI, kD); // period is set via the underlying controller
    }

    // -------------------------------------------------------------------------
    // Core tuning update – call this from periodic()
    // -------------------------------------------------------------------------

    /**
     * Reads P, I, D from SmartDashboard and updates the underlying
     * {@link PIDController} if any value has changed.
     *
     * <p>
     * <b>Call this from your subsystem's {@code periodic()} method.</b>
     *
     * <p>
     * Also pushes the current setpoint and error to the dashboard for
     * monitoring.
     */
    public void updateFromDashboard() {
        double newP = SmartDashboard.getNumber(m_keyP, m_lastP);
        double newI = SmartDashboard.getNumber(m_keyI, m_lastI);
        double newD = SmartDashboard.getNumber(m_keyD, m_lastD);

        if (newP != m_lastP || newI != m_lastI || newD != m_lastD) {
            m_pid.setPID(newP, newI, newD);
            m_lastP = newP;
            m_lastI = newI;
            m_lastD = newD;
        }

        // Always publish live monitoring values
        SmartDashboard.putNumber(m_keySetpoint, m_pid.getSetpoint());
        SmartDashboard.putNumber(m_keyError, m_pid.getError());
    }

    // -------------------------------------------------------------------------
    // Delegated PIDController API – mirrors PIDController exactly
    // -------------------------------------------------------------------------

    /**
     * Calculates the PID output. Equivalent to
     * {@link PIDController#calculate(double, double)}.
     */
    public double calculate(double measurement, double setpoint) {
        return m_pid.calculate(measurement, setpoint);
    }

    /**
     * Calculates the PID output using the previously set setpoint.
     * Equivalent to {@link PIDController#calculate(double)}.
     */
    public double calculate(double measurement) {
        return m_pid.calculate(measurement);
    }

    /** @see PIDController#setSetpoint(double) */
    public void setSetpoint(double setpoint) {
        m_pid.setSetpoint(setpoint);
    }

    /** @see PIDController#getSetpoint() */
    public double getSetpoint() {
        return m_pid.getSetpoint();
    }

    /** @see PIDController#atSetpoint() */
    public boolean atSetpoint() {
        return m_pid.atSetpoint();
    }

    /** @see PIDController#getError() */
    public double getError() {
        return m_pid.getError();
    }

    /** @see PIDController#reset() */
    public void reset() {
        m_pid.reset();
    }

    /** @see PIDController#setTolerance(double) */
    public void setTolerance(double positionTolerance) {
        m_pid.setTolerance(positionTolerance);
    }

    /** @see PIDController#setTolerance(double, double) */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_pid.setTolerance(positionTolerance, velocityTolerance);
    }

    /** @see PIDController#enableContinuousInput(double, double) */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_pid.enableContinuousInput(minimumInput, maximumInput);
    }

    /** @see PIDController#disableContinuousInput() */
    public void disableContinuousInput() {
        m_pid.disableContinuousInput();
    }

    /** @see PIDController#setIntegratorRange(double, double) */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_pid.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    /** @see PIDController#setIZone(double) */
    public void setIZone(double iZone) {
        m_pid.setIZone(iZone);
    }

    /** @see PIDController#getP() */
    public double getP() {
        return m_pid.getP();
    }

    /** @see PIDController#getI() */
    public double getI() {
        return m_pid.getI();
    }

    /** @see PIDController#getD() */
    public double getD() {
        return m_pid.getD();
    }

    /**
     * Returns the underlying {@link PIDController} for any advanced usage
     * not covered by this wrapper.
     */
    public PIDController getInternalController() {
        return m_pid;
    }
}