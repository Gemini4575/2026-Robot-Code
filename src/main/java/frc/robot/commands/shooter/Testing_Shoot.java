package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.BeamBreak;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class Testing_Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final BeamBreak beamBreak;
    private final AdvancerSubsystem advancer;
    private final Supplier<Pose2d> poseSupplier;

    public Testing_Shoot(ShooterSubsystem shooterSubsystem, BeamBreak beamBreak, AdvancerSubsystem advancerSubsystem,
            Supplier<Pose2d> poseSupplier) {
        this.shooter = shooterSubsystem;
        this.advancer = advancerSubsystem;
        this.beamBreak = beamBreak;
        this.poseSupplier = poseSupplier;
        addRequirements(shooterSubsystem, beamBreak);
    }

    private boolean firstRun = true;

    @Override
    public void initialize() {
        firstRun = true;
    }

    @Override
    public void execute() {
        if (shooter.runShooterAtDistance(poseSupplier.get())) {
            advancer.advance();
            firstRun = false;
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
        return /* beamBreak.getShooter() == */ false;
    }
}
