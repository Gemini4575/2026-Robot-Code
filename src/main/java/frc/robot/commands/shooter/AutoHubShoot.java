package frc.robot.commands.shooter;

import frc.robot.commands.shooter.base.AutoShootBase;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class AutoHubShoot extends AutoShootBase {
    public AutoHubShoot(ShooterSubsystem a, AdvancerSubsystem ds) {
        super(a, ds, 3050);
    }

}
