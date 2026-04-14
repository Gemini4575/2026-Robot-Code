package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.commands.shooter.base.TelopShootBase;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class TelopAlleniceWallShot extends TelopShootBase {
    public TelopAlleniceWallShot(ShooterSubsystem AdvancerSubsystem, AdvancerSubsystem ShooterSubsystem) {
        super(AdvancerSubsystem, ShooterSubsystem, Constants.ShooterRPMConstants.ALLIANCE_WALL_SHOT);
    }

}
