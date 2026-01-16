package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class ResetLocationCommand extends Command {

    private final DrivetrainIO driveTrain;
    private final Pose2d pose;

    private boolean hasRun = false;

    public ResetLocationCommand(DrivetrainIO driveTrain, Pose2d pose) {
        this.driveTrain = driveTrain;
        this.pose = pose;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        hasRun = false;
    }

    @Override
    public boolean isFinished() {
        return hasRun;
    }

    @Override
    public void execute() {
        if (!hasRun) {
            driveTrain.ResetGyro();
            driveTrain.resetPose(pose);
            hasRun = true;
        }
    }

}
