package frc.robot.commands.shooter;

import frc.robot.commands.shooter.base.AutoShootBase;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class AutoShootMiddle extends AutoShootBase {
    public AutoShootMiddle(ShooterSubsystem a, AdvancerSubsystem ds) {
        super(a, ds, 4000);
    }

}
