package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class TimedTestWheelTurn extends Command {
    private final long durationMillis;
    private final DrivetrainIO driveTrain;

    private long startTime;
    private boolean positiveTurnDirection = true;
    private long turnStart;

    private static final long TURN_DURATION_MS = 500;

    public TimedTestWheelTurn(DrivetrainIO driveTrain, long durationMillis) {
        this.driveTrain = driveTrain;
        this.durationMillis = durationMillis;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        turnStart = startTime;
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - turnStart > TURN_DURATION_MS) {
            positiveTurnDirection = !positiveTurnDirection;
            turnStart = currentTime;
        }
        driveTrain.setModuleStates(new SwerveModuleState[] {
                generateTargetState(), generateTargetState(), generateTargetState(), generateTargetState() });
    }

    private SwerveModuleState generateTargetState() {
        return new SwerveModuleState(0, Rotation2d.fromDegrees(positiveTurnDirection ? 89 : 0));
    }

    @Override
    public boolean isFinished() {
        var done = System.currentTimeMillis() - startTime >= durationMillis;
        if (done) {
            driveTrain.drive(0, 0, 0, false);
        }
        return done;
    }
}
