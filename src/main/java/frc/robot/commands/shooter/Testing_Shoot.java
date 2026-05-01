package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.BeamBreak;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class Testing_Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final AdvancerSubsystem advancer;

    public Testing_Shoot(ShooterSubsystem shooterSubsystem, BeamBreak beamBreak, AdvancerSubsystem advancerSubsystem,
            Supplier<Pose2d> poseSupplier) {
        this.shooter = shooterSubsystem;
        this.advancer = advancerSubsystem;
        addRequirements(shooterSubsystem, beamBreak);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (shooter.runShooter()) {
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
        return false;
    }
}
