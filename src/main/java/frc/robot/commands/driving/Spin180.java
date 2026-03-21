package frc.robot.commands.driving;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class Spin180 extends Command {
    DrivetrainIO d;
    boolean isFinished;

    private ProfiledPIDController rotation = new ProfiledPIDController(
            0.1,
            0,
            0,
            new TrapezoidProfile.Constraints(2, 2));

    public Spin180(DrivetrainIO d) {
        this.d = d;
    }

    @Override
    public void initialize() {
        rotation.enableContinuousInput(-180, 180);
        isFinished = false;
    }

    double startAngle = 0.0;
    double Rotate_Rot = 0.0;
    boolean first = true;

    private void first() {
        if (first) {
            startAngle = d.getAngle();
            first = false;
        }
    }

    public void end() {
        first = true;
    }

    // This is origanly with -180 to 180 bounds but we found that we ran into a
    // problem when it would flip and the robot would ocsolate due to the signum
    // calculation so we just made it contiues and just scalled down the values we
    // gave the PID loop
    public boolean rotate(Rotation2d targetAngle) {
        first();
        double currentAngle = d.getAngle();
        double targetDegrees = startAngle + targetAngle.getDegrees();

        // Normalize error to [-180, 180] so we always take the shortest path
        double error = targetDegrees - currentAngle;
        while (error > 360)
            error -= 360;
        while (error < 0)
            error += 360;

        SmartDashboard.putNumber("[DriveTrain]currentAngle", currentAngle);
        SmartDashboard.putNumber("[DriveTrain]targetDegrees", targetDegrees);
        SmartDashboard.putNumber("[DriveTrain]error", error);

        if (Math.abs(error) < 5.0) {
            d.Rotate_Rot(0);
            first = true;
            return true;
        }

        Rotate_Rot = 0.2;
        // d.Rotate_Rot(Rotate_Rot);
        d.drive(0, 0, Rotate_Rot, false);
        return false;
    }

    @SuppressWarnings("static-access")
    @Override
    public void execute() {
        isFinished = rotate(new Rotation2d().fromDegrees(180));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean ds) {
        if (ds) {
            end();
        }
    }
}
