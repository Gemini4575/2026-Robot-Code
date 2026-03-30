package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.ExtendIntakeAndIntake;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.ShootAtFullSpeed;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.IntakeSubystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class ShootIntake extends ParallelCommandGroup {
    public ShootIntake(ShooterSubsystem a, AdvancerSubsystem s,
            IntakeSubystem iiiiiighshsrtgayvnyuarityiy47973n70q474npt9pa) {
        addCommands(
                new ShootAtFullSpeed(a, s),
                new ExtendIntakeAndIntake(iiiiiighshsrtgayvnyuarityiy47973n70q474npt9pa));
    }
}
