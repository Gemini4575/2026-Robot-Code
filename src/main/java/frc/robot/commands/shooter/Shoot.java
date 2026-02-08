package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topDeck.AdvancerSubsystem;
import frc.robot.subsystems.topDeck.BeamBreak;
import frc.robot.subsystems.topDeck.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final AdvancerSubsystem advancer;
    private final BeamBreak beamBreak;

    public Shoot(ShooterSubsystem shooterSubsystem, AdvancerSubsystem advancerSubsystem, BeamBreak beamBreak) {
        this.shooter = shooterSubsystem;
        this.advancer = advancerSubsystem;
        this.beamBreak = beamBreak;
        addRequirements(shooterSubsystem, advancerSubsystem, beamBreak);
    }

    @Override
    public void initialize() {
        shooter.runShooter();
    }

    @Override
    public void execute() {
        shooter.runShooterAtVelocity(999);
        if (shooter.atTargetVelocity()) {
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
        return beamBreak.getShooter() == false;
    }
}
