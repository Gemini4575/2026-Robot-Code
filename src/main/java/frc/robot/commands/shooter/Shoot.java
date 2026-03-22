package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.BeamBreak;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final AdvancerSubsystem advancer;
    private final BeamBreak beamBreak;
    private final Supplier<Pose2d> poseSupplier;

    private boolean shooterReady = false;

    /**
     * Shoots a ball using interpolated RPM based on the robot's distance to the hub.
     *
     * @param shooterSubsystem  the shooter subsystem
     * @param advancerSubsystem the advancer subsystem
     * @param beamBreak         the beam break sensor
     * @param poseSupplier      supplier for the robot's current field pose (e.g. drivetrain::getPose)
     */
    public Shoot(ShooterSubsystem shooterSubsystem, AdvancerSubsystem advancerSubsystem,
            BeamBreak beamBreak, Supplier<Pose2d> poseSupplier) {
        this.shooter = shooterSubsystem;
        this.advancer = advancerSubsystem;
        this.beamBreak = beamBreak;
        this.poseSupplier = poseSupplier;
        addRequirements(shooterSubsystem, advancerSubsystem, beamBreak);
    }

    @Override
    public void initialize() {
        shooterReady = false;
    }

    @Override
    public void execute() {
        // Spin up to the interpolated RPM for current distance; returns true when on target
        shooterReady = shooter.runShooterAtDistance(poseSupplier.get());

        if (shooterReady) {
            advancer.advance();
        } else {
            advancer.stopAdvancer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        advancer.stopAdvancer();
    }

    @Override
    public boolean isFinished() {
        // Stop once the ball has cleared the shooter beam break
        return beamBreak.getShooter();
    }
}
