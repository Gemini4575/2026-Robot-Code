package frc.robot.commands.driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class FaceTowardsCoordinates extends Command {

    private static final double TURN_PRECISION = 5 * Math.PI / 180; // 5 degrees tolerance
    private static final double MAX_ANGULAR_SPEED = Math.PI / 2; // radians per second
    private static final double MIN_ANGULAR_SPEED = 0.1; // minimum rotation speed to overcome friction

    private final DrivetrainIO driveSubsystem;
    private final Translation2d targetCoordinates;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    
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
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Get driver inputs for translation
        double xSpeed = xSpeedSupplier.getAsDouble();
        double ySpeed = ySpeedSupplier.getAsDouble();
        
        // Calculate the angle from robot to target coordinates
        double deltaX = targetCoordinates.getX() - currentPose.getX();
        double deltaY = targetCoordinates.getY() - currentPose.getY();
        double targetAngle = Math.atan2(deltaY, deltaX);
        
        // Calculate angular difference
        double currentAngle = currentPose.getRotation().getRadians();
        double rawAngularDiff = targetAngle - currentAngle;
        double angularDiff = optimizeAngle(rawAngularDiff);
        
        // Calculate rotation speed based on angular difference
        double rotationSpeed = calcAngularSpeed(angularDiff);
        
        // Drive with driver-controlled translation AND automatic rotation
        driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true);
        
        // Telemetry for debugging
        SmartDashboard.putNumber("[FaceTowards] Target Angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("[FaceTowards] Current Angle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("[FaceTowards] Angular Diff", Math.toDegrees(angularDiff));
        SmartDashboard.putNumber("[FaceTowards] Rotation Speed", rotationSpeed);
        SmartDashboard.putNumber("[FaceTowards] Driver X Speed", xSpeed);
        SmartDashboard.putNumber("[FaceTowards] Driver Y Speed", ySpeed);
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        driveSubsystem.drive(0, 0, 0, true);
    }

    /**
     * Optimizes angle to turn in the shortest direction
     */
    private double optimizeAngle(double rawAngularDiff) {
        if (rawAngularDiff > Math.PI) {
            return rawAngularDiff - 2.0 * Math.PI;
        }
        if (rawAngularDiff < -Math.PI) {
            return rawAngularDiff + 2.0 * Math.PI;
        }
        return rawAngularDiff;
    }

    /**
     * Calculates angular speed with limits and proportional control
     */
    private double calcAngularSpeed(double angularDiff) {
        // Proportional control - rotate faster when further from target
        double rawSpeed = angularDiff * 2.0; // Proportional gain
        
        // Clamp to max speed
        double clampedSpeed = Math.max(-MAX_ANGULAR_SPEED, 
                                      Math.min(MAX_ANGULAR_SPEED, rawSpeed));
        
        // Apply minimum speed to overcome friction when not aligned
        if (Math.abs(angularDiff) > TURN_PRECISION) {
            if (clampedSpeed > 0 && clampedSpeed < MIN_ANGULAR_SPEED) {
                clampedSpeed = MIN_ANGULAR_SPEED;
            } else if (clampedSpeed < 0 && clampedSpeed > -MIN_ANGULAR_SPEED) {
                clampedSpeed = -MIN_ANGULAR_SPEED;
            }
        } else {
            clampedSpeed = 0.0;
        }
        
        return clampedSpeed;
    }
}
