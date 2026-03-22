package frc.robot.commands.driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class FaceTowardsCoordinates extends Command {

    // Hub coordinates — blue alliance origin (bottom-left of field)
    private static final Translation2d BLUE_HUB = new Translation2d(11.914, 4.051);
    private static final Translation2d RED_HUB = new Translation2d(5.634, 4.051);

    // Tuning constants — adjust these on the robot
    private static final double KP = 10.0; // start here, increase if too slow
    private static final double KI = 0.0; // leave at 0 unless it never quite reaches target
    private static final double KD = 0.1; // helps damp oscillation
    private static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2.0 * Math.PI; // 360 deg/s
    private static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = 4.0 * Math.PI; // 720 deg/s^2
    private static final double TOLERANCE_DEGREES = 5.0;

    private final DrivetrainIO driveSubsystem;
    private final Translation2d targetCoordinates;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;

    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            KP, KI, KD,
            new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
                    MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));

    // Constructor for explicit coordinates
    public FaceTowardsCoordinates(DrivetrainIO driveSubsystem, double targetX, double targetY,
            DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.targetCoordinates = new Translation2d(targetX, targetY);
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        configureController();
        addRequirements(driveSubsystem);
    }

    // Alliance-aware constructor — automatically targets blue or red hub
    public FaceTowardsCoordinates(DrivetrainIO driveSubsystem,
            DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        var alliance = DriverStation.getAlliance();
        this.targetCoordinates = (alliance.isPresent() && alliance.get() == Alliance.Red)
                ? RED_HUB
                : BLUE_HUB;
        configureController();
        addRequirements(driveSubsystem);
    }

    private void configureController() {
        // enableContinuousInput tells the PID that -PI and PI are the same point
        // so it always takes the shortest rotation path automatically
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(TOLERANCE_DEGREES));
    }

    @Override
    public void initialize() {
        // Reset from current heading so there's no sudden jump on first loop
        rotationController.reset(driveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        double deltaX = targetCoordinates.getX() - currentPose.getX();
        double deltaY = targetCoordinates.getY() - currentPose.getY();

        // Rotation2d(deltaX, deltaY) is equivalent to atan2(deltaY, deltaX)
        // but cleaner and avoids manual normalization
        Rotation2d targetAngle = new Rotation2d(deltaX, deltaY);

        // PID calculates rotation speed — enableContinuousInput handles wraparound
        double rotationSpeed = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetAngle.getRadians());

        driveSubsystem.drive(
                xSpeedSupplier.getAsDouble(),
                ySpeedSupplier.getAsDouble(),
                rotationSpeed,
                true);

        SmartDashboard.putNumber("[FaceTowards] Target Angle (deg)", targetAngle.getDegrees());
        SmartDashboard.putNumber("[FaceTowards] Current Angle (deg)", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("[FaceTowards] Error (deg)",
                Units.radiansToDegrees(rotationController.getPositionError()));
        SmartDashboard.putNumber("[FaceTowards] Rotation Output", rotationSpeed);
        SmartDashboard.putBoolean("[FaceTowards] At Target", rotationController.atSetpoint());
    }

    @Override
    public boolean isFinished() {
        return rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);
    }
}