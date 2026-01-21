
package frc.robot.commands.driving;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class FaceTowardsCoordinates extends Command {

    private static final double TURN_PRECISION = Units.degreesToRadians(5); // 5 degrees tolerance
    private static final double MAX_ANGULAR_SPEED = 2.0; // radians per second (for profiling)
    private static final double MAX_ANGULAR_ACCELERATION = 4.0; // radians per second^2 (for profiling)
    private static final double DEADBAND_THRESHOLD = Units.degreesToRadians(3); // Don't move if within 3 degrees
    private static final double FLIP_ZONE_THRESHOLD = Units.degreesToRadians(160); // Detect if we're in the danger zone
    
    // TreeMap that scales rotation output based on distance to target
    // Key: distance in meters, Value: max rotation output (0.0 to 1.0)
    private static final TreeMap<Double, Double> MAX_ROTATION_OUTPUTS = new TreeMap<>(
        Map.of(
            0.0, 0.4,   // Very close: slow rotation
            0.25, 0.8,   // Close: moderate rotation
            1.0, 0.7,   // Medium distance: faster rotation
            2.0, 1.0    // Far: full speed rotation
        )
    );

    private final DrivetrainIO driveSubsystem;
    private final Translation2d targetCoordinates;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    
    private final ProfiledPIDController rotationController;
    private double lastRotationDirection = 0.0; // Track which way we were rotating
    private int executeCount = 0; // Count how many times execute() has run
    
    /**
     * Creates a command that continuously rotates the robot to face towards specified coordinates
     * while allowing driver control of translation (movement)
     * @param driveSubsystem The drivetrain subsystem
     * @param targetX Target X coordinate in meters
     * @param targetY Target Y coordinate in meters
     * @param xSpeedSupplier Supplier for X speed from joystick (-1.0 to 1.0)
     * @param ySpeedSupplier Supplier for Y speed from joystick (-1.0 to 1.0)
     */
    public FaceTowardsCoordinates(DrivetrainIO driveSubsystem, double targetX, double targetY,
                                   DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.targetCoordinates = new Translation2d(targetX, targetY);
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        
        // Create PID controller with motion profiling
        rotationController = new ProfiledPIDController(
            4.0,  // kP - tune this value (lower since we're outputting -1 to 1)
            0.0,  // kI
            0.05, // kD - tune this value
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION)
        );
        
        // CRITICAL: This prevents flipping between -180 and +180
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set tolerance for isFinished()
        rotationController.setTolerance(TURN_PRECISION);
        
        // Clamp integrator to max rotation output
        rotationController.setIntegratorRange(-1.0, 1.0);
        
        addRequirements(driveSubsystem);
    }
    
    /**
     * Alternative constructor using Translation2d
     */
    public FaceTowardsCoordinates(DrivetrainIO driveSubsystem, Translation2d targetCoordinates,
                                   DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.targetCoordinates = targetCoordinates;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        
        // Create PID controller with motion profiling
        rotationController = new ProfiledPIDController(
            3.0,  // kP - tune this value
            0.0,  // kI
            0.1,  // kD - tune this value
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION)
        );
        
        // CRITICAL: This prevents flipping between -180 and +180
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set tolerance for isFinished()
        rotationController.setTolerance(TURN_PRECISION);
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset the PID controller when command starts
        rotationController.reset(driveSubsystem.getPose().getRotation().getRadians());
        lastRotationDirection = 0.0;
        executeCount = 0;
    }

    double targetAngle = 0.0;
    double currentAngle = 0.0;

    @Override
    public void execute() {
        executeCount++;
        
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Get driver inputs for translation
        double xSpeed = xSpeedSupplier.getAsDouble();
        double ySpeed = ySpeedSupplier.getAsDouble();
        
        // Calculate the angle from robot to target coordinates
        double deltaX = targetCoordinates.getX() - currentPose.getX();
        double deltaY = targetCoordinates.getY() - currentPose.getY();
        targetAngle = Math.atan2(deltaY, deltaX) - Math.PI / 2;
        
        // Calculate distance to target
        double distanceToTarget = Math.hypot(deltaX, deltaY);
        
        // Get scaled max rotation output based on distance
        double maxRotationOutput = getMaxRotationOutput(distanceToTarget);
        
        // Get current angle
        currentAngle = currentPose.getRotation().getRadians();
        
        // Get the error from the PID controller
        double error = rotationController.getPositionError();
        
        // Calculate rotation speed using PID controller
        // The PID controller handles continuous input automatically
        double rawRotationSpeed = -rotationController.calculate(currentAngle, targetAngle); // Inverted!
        
        // CRITICAL FIX: Prevent sign flipping when near ±180°
        double absError = Math.abs(error);
        
        // If we're in the flip zone (near 180°) and we were already rotating
        if (absError > FLIP_ZONE_THRESHOLD && lastRotationDirection != 0.0) {
            // Continue rotating in the same direction we were going
            rawRotationSpeed = Math.abs(rawRotationSpeed) * Math.signum(lastRotationDirection);
        }
        
        // Apply deadband - don't rotate if we're very close to target
        if (absError < DEADBAND_THRESHOLD) {
            rawRotationSpeed = 0.0;
        }
        
        // Remember which direction we're rotating
        if (Math.abs(rawRotationSpeed) > 0.01) {
            lastRotationDirection = rawRotationSpeed;
        }
        
        // Clamp output to scaled max rotation based on distance
        double rotationSpeed = Math.max(-maxRotationOutput, 
                                       Math.min(maxRotationOutput, rawRotationSpeed));
        
        // Drive with driver-controlled translation AND automatic rotation
        driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true);
        
        // Telemetry for debugging
        SmartDashboard.putNumber("[FaceTowards] Target Angle (deg)", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("[FaceTowards] Current Angle (deg)", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("[FaceTowards] Error (deg)", Math.toDegrees(error));
        SmartDashboard.putNumber("[FaceTowards] Abs Error (deg)", Math.toDegrees(absError));
        SmartDashboard.putNumber("[FaceTowards] Distance to Target", distanceToTarget);
        SmartDashboard.putNumber("[FaceTowards] Max Rotation Output", maxRotationOutput);
        SmartDashboard.putNumber("[FaceTowards] Raw Rotation Speed", rawRotationSpeed);
        SmartDashboard.putNumber("[FaceTowards] Clamped Rotation Speed", rotationSpeed);
        SmartDashboard.putNumber("[FaceTowards] Last Direction", lastRotationDirection);
        SmartDashboard.putNumber("[FaceTowards] Driver X Speed", xSpeed);
        SmartDashboard.putNumber("[FaceTowards] Driver Y Speed", ySpeed);
        SmartDashboard.putBoolean("[FaceTowards] At Setpoint", rotationController.atSetpoint());
        SmartDashboard.putBoolean("[FaceTowards] In Flip Zone", absError > FLIP_ZONE_THRESHOLD);
        SmartDashboard.putNumber("[FaceTowards] Execute Count", executeCount);
    }

    @Override
    public boolean isFinished() {
        // Don't finish for the first 5 cycles to ensure PID has time to initialize
        // Then finish when at setpoint
        return Math.abs(Math.toDegrees(targetAngle) - Math.toDegrees(currentAngle)) <= 7;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        driveSubsystem.drive(0, 0, 0, true);
    }
    
    /**
     * Gets the maximum rotation output based on distance to target
     * Uses TreeMap similar to DriveToLocation's MAX_SPEEDS
     * 
     * @param distanceToTarget Distance in meters
     * @return Maximum rotation output (0.0 to 1.0)
     */
    private double getMaxRotationOutput(double distanceToTarget) {
        var speedEntry = MAX_ROTATION_OUTPUTS.floorEntry(distanceToTarget);
        return speedEntry == null ? MAX_ROTATION_OUTPUTS.firstEntry().getValue() : speedEntry.getValue();
    }
}