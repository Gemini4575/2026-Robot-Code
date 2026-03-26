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
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class FaceTowardsCoordinates extends Command {

    private static final Translation2d BLUE_HUB = new Translation2d(11.914, 4.051);
    private static final Translation2d RED_HUB = new Translation2d(5.634, 4.051);

    // KP operates on radians, output is divided by kModuleMaxAngularVelocity to get
    // -1..1
    // At 90 deg error (1.57 rad): output % = KP * 1.57 / 27.7
    // KP=3 → ~17% speed at 90 deg, scales to 0 at target
    private static final double KP = 5.0;
    private static final double KI = 0.0;
    private static final double KD = 0.05;
    private static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI; // 180 deg/s
    private static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = 2.0 * Math.PI; // 360 deg/s^2
    private static final double TOLERANCE_DEGREES = 2.0;

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

    // Alliance-aware constructor
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
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(TOLERANCE_DEGREES));
    }

    private Rotation2d getTargetAngle() {
        Pose2d currentPose = driveSubsystem.getPose();
        double deltaX = targetCoordinates.getX() - currentPose.getX();
        double deltaY = targetCoordinates.getY() - currentPose.getY();
        return new Rotation2d(deltaX, deltaY);
    }

    @Override
    public void initialize() {
        rotationController.reset(driveSubsystem.getPose().getRotation().getRadians(), 0);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        Rotation2d targetAngle = getTargetAngle();

        // PID output is rad/s — divide by max angular velocity to get -1..1 for drive()
        double rotationRadPerSec = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetAngle.getRadians());
        double rotationSpeed = Math.max(-1.0, Math.min(1.0,
                rotationRadPerSec / SwerveConstants.kModuleMaxAngularVelocity));

        driveSubsystem.drive(
                xSpeedSupplier.getAsDouble(),
                ySpeedSupplier.getAsDouble(),
                rotationSpeed,
                true);

        SmartDashboard.putNumber("[FaceTowards] Target Angle (deg)", targetAngle.getDegrees());
        SmartDashboard.putNumber("[FaceTowards] Current Angle (deg)", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("[FaceTowards] Error (deg)",
                Units.radiansToDegrees(rotationController.getPositionError()));
        SmartDashboard.putNumber("[FaceTowards] Rotation rad/s", rotationRadPerSec);
        SmartDashboard.putNumber("[FaceTowards] Rotation Output %", rotationSpeed);
        SmartDashboard.putBoolean("[FaceTowards] At Goal", rotationController.atGoal()); // atGoal not atSetpoint!
    }

    @Override
    public boolean isFinished() {
        return rotationController.atGoal(); // atGoal checks final target, atSetpoint checks current profile position
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);
    }
}