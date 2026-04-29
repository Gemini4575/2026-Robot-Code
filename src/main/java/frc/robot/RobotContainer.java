// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.shooter.TelopLadderShoot;
import frc.robot.commands.shooter.TelopTrenchShoot;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.commands.ShootIntake;
import frc.robot.commands.advancer.Advance;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.commands.auto.shooter.ShootFromAutoMiddle;
import frc.robot.commands.auto.shooter.ShootFromDepot;
import frc.robot.commands.auto.shooter.ShootFromTrench;
import frc.robot.commands.auto.shooter.SpinUpShooter;
import frc.robot.commands.climber.ClimbTelop;
import frc.robot.commands.climber.ClimbTo0;
import frc.robot.commands.climber.DownToClimb;
import frc.robot.commands.climber.UpToClimb;
import frc.robot.commands.driving.AlineWheels;
import frc.robot.commands.driving.DriveForSecondsBackwards;
import frc.robot.commands.driving.DriveForSecondsForwards;
import frc.robot.commands.driving.DriveForSecondsLeft;
import frc.robot.commands.driving.DriveForSecondsRight;
import frc.robot.commands.driving.DriveForSecondsSlow;
import frc.robot.commands.driving.DriveToLocation;
import frc.robot.commands.driving.FaceTowardsCoordinates;
import frc.robot.commands.driving.ResetLocationCommand;
import frc.robot.commands.driving.RotateSomeAmount;
import frc.robot.commands.driving.Spin180;
import frc.robot.commands.driving.Stop;
import frc.robot.commands.driving.StopForever;
import frc.robot.commands.driving.TeleopSwerve;
import frc.robot.commands.driving.TimedTestDrive;
import frc.robot.commands.driving.XTheWheels;
import frc.robot.commands.driving.XTheWheelsTimed;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.ExtendIntakeAndIntake;
import frc.robot.commands.intake.ExtendOrRectactIntake;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.AutoAllienceWallShoot;
import frc.robot.commands.shooter.AutoHubShoot;
import frc.robot.commands.shooter.ReverseShoot;
import frc.robot.commands.shooter.TelopAlleniceWallShot;
import frc.robot.commands.shooter.AutoShootMiddle;
import frc.robot.commands.shooter.Testing_Shoot;
import frc.robot.commands.smartDashBoard.SendNote;
import frc.robot.model.MetricName;
import frc.robot.model.PathContainer;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;
import frc.robot.service.MetricService;
import frc.robot.subsystems.pathfinding.Vision;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.BeamBreak;
import frc.robot.subsystems.topdeck.ClimberSubsystem;
import frc.robot.subsystems.topdeck.ClimberSubsystem;
// import frc.robot.subsystems.topdeck.HoodSubsystem;
import frc.robot.subsystems.topdeck.IntakeSubystem;
import frc.robot.subsystems.topdeck.LimitSwitchSubsystem;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

import static frc.robot.Constants.JoystickConstants.*;
import static frc.robot.Constants.HoodConstants.*;

import java.util.Map;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  ShuffleboardTab DIO_tab = Shuffleboard.getTab("DIO");

  ShuffleboardTab note_tab = Shuffleboard.getTab("Notes");

  GenericEntry note_entry = note_tab.add("Note", "")
      .withPosition(0, 0)
      .withSize(2, 1)
      .getEntry();

  GenericEntry DIO_entry = DIO_tab.add("DIO 0", false)
      .withPosition(0, 0)
      .withSize(2, 1)
      .getEntry();

  GenericEntry DIO_entry1 = DIO_tab.add("DIO 1", false)
      .withPosition(2, 0)
      .withSize(2, 1)
      .getEntry();

  GenericEntry DIO_entry2 = DIO_tab.add("DIO 2", false)
      .withPosition(4, 0)
      .withSize(2, 1)
      .getEntry();

  GenericEntry DIO_entry3 = DIO_tab.add("DIO 3", false)
      .withPosition(6, 0)
      .withSize(2, 1)
      .getEntry();

  ShuffleboardLayout elevatorCommands = note_tab
      .getLayout("Send", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withProperties(Map.of("Label position", "HIDDEN"));

  Field2d visionPoseEstimate = new Field2d();
  Field2d overallPoseEstimate = new Field2d();

  Timer autoTimer = new Timer();

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  private final Joystick testing = new Joystick(3);
  private final Joystick climber = new Joystick(2);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, BACK_BUTTON);
  private final Trigger Slow = new Trigger(new JoystickButton(driver, 7)
      .and(new JoystickButton(driver, 12)))
      .or(new JoystickButton(operator, START_BUTTON));

  /* Pathplanner stuff */
  private final SendableChooser<Command> PathplannerautoChoosers;
  // @SuppressWarnings("unused")
  // private final SendableChooser<Command> autoChooser;

  /* Subsystems */
  private final ShooterSubsystem S = new ShooterSubsystem();
  private final ClimberSubsystem C = new ClimberSubsystem();
  private final IntakeSubystem I = new IntakeSubystem();
  private final BeamBreak b = new BeamBreak();
  private final LimitSwitchSubsystem limitSwitches = new LimitSwitchSubsystem();
  private final AdvancerSubsystem A = new AdvancerSubsystem();
  // private final HoodSubsystem H = new HoodSubsystem();
  private final DrivetrainIO D = new DrivetrainIO();
  private Vision V;
  // private final Lidar lidar = new Lidar();

  // private final LaserCan lc;

  public RobotContainer() {

    // WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    note_entry.setString("RobotContainer initialized");
    try {
      V = new Vision();
    } catch (Exception e) {
      System.out.println("Vision subsystem failed to initialize: " + e);
    }

    // lc = initLaserCAN();

    SmartDashboard.putData("[Robot]Vision Pose Estimate", visionPoseEstimate);
    SmartDashboard.putData("[Robot]Overall Pose Estimate", overallPoseEstimate);
    NamedCommands.registerCommand("Aline Wheels", new AlineWheels(D));
    NamedCommands.registerCommand("Shoot From Depot", new ShootFromDepot(S, A, b).withTimeout(5));
    NamedCommands.registerCommand("Shoot From Trench", new ShootFromTrench(S, A, b).withTimeout(5));
    NamedCommands.registerCommand("Shoot From Hub", new AutoHubShoot(S, A));
    NamedCommands.registerCommand("Drop Intake", new ExtendIntake(I));
    NamedCommands.registerCommand("Spin Up Shooter", new SpinUpShooter(S));
    NamedCommands.registerCommand("Down To Climb", new DownToClimb(C));
    NamedCommands.registerCommand("Climb", new UpToClimb(C));
    NamedCommands.registerCommand("Stop", new Stop(D));
    NamedCommands.registerCommand("X The Wheels", new XTheWheelsTimed(D));
    NamedCommands.registerCommand("Shoot From Auto Middle", new ShootFromAutoMiddle(S, A));
    NamedCommands.registerCommand("Spin 180", new Spin180(D));
    NamedCommands.registerCommand("Face Hub", new FaceTowardsCoordinates(D,
        11.914,
        4.051,
        () -> 0,
        () -> 0));
    PathplannerautoChoosers = AutoBuilder.buildAutoChooser();
    // PathplannerautoChoosers.addOption("quazisestatic fowerd",
    // D.sysIdQuasistatic(Direction.kForward));
    // PathplannerautoChoosers.addOption("quazisestatic reverse",
    // D.sysIdQuasistatic(Direction.kReverse));
    // PathplannerautoChoosers.addOption("dynamic fowerd",
    // D.sysIdDynamic(Direction.kForward));
    // PathplannerautoChoosers.addOption("dynamic reverse",
    // D.sysIdDynamic(Direction.kReverse));

    PathplannerautoChoosers.addOption("Go To outpost and shoot",
        new AlineWheels(D)
            .andThen(new DriveForSecondsForwards(D, 3.75).alongWith(new ExtendIntake(I)))
            .andThen(new WaitCommand(5).alongWith(new StopForever(D).withTimeout(5)))
            .andThen(new DriveForSecondsBackwards(D, 4 / 2))
            .andThen(new DriveForSecondsLeft(D, 0.5))
            .andThen(new RotateSomeAmount(D, () -> 240))
            .andThen(new TelopAlleniceWallShot(S, A).alongWith(new StopForever(D))));
    // .andThen(new DriveForSecondsBackwards(D, 4.15 / 2))
    // .andThen(new DriveForSecondsLeft(D, 4.0))
    // .andThen(new Testing_Shoot(S, b, A, D::getPose).withTimeout(10)));
    PathplannerautoChoosers.addOption("Go To the depot and shoot",
        new AlineWheels(D)
            .andThen(new DriveForSecondsRight(D, 2.05))
            .andThen(new DriveForSecondsBackwards(D, 1.25))
            .andThen(new Spin180(D))
            .andThen(new DriveForSecondsForwards(D, 3).alongWith(new ExtendIntakeAndIntake(I).withTimeout(3.5)))
            .andThen(new DriveForSecondsBackwards(D, 1).alongWith(new ExtendIntakeAndIntake(I).withTimeout(2)))
            .andThen(new RotateSomeAmount(D, () -> 160))
            .andThen(new AutoAllienceWallShoot(S, A).alongWith(new Stop(D))));
    PathplannerautoChoosers.addOption("Shoot", new Testing_Shoot(S, b, A, D::getPose).withTimeout(5));
    // autoChooser = new AutoCommandFactory(D, lc).generateAutoOptions();
    SmartDashboard.putData("[Robot]Auto Chosers", PathplannerautoChoosers);
    PathfindingCommand.warmupCommand().schedule();
    // Configure the trigger bindings
    configureBindings();
  }

  private LaserCan initLaserCAN() {
    LaserCan lc = new LaserCan(1);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Laser CanConfiguration failed! " + e);
    }
    return lc;
  }

  private void configureBindings() {
    /* Debug */
    elevatorCommands.add("Send Note", new SendNote(note_entry.getString("")));
    /* Driver Controls */
    zeroGyro.onTrue(new InstantCommand(() -> D.ResetGyro()));
    D.setDefaultCommand(
        new TeleopSwerve(
            D,
            () -> -driver.getRawAxis(LEFT_Y_AXIS),
            () -> -driver.getRawAxis(LEFT_X_AXIS),
            () -> -driver.getRawAxis(RIGHT_X_AXIS),
            Slow,
            () -> driver.getPOV()));
    // new JoystickButton(driver, RED_BUTTON)
    // .onTrue(new Spin180(D).asProxy());

    new JoystickButton(testing, YELLOW_BUTTON)
        .onTrue(new FaceTowardsCoordinates(D,
            () -> -driver.getRawAxis(LEFT_Y_AXIS),
            () -> driver.getRawAxis(LEFT_X_AXIS)));

    // new JoystickButton(driver, YELLOW_BUTTON).onTrue(new TimedTestDrive(D, 2000,
    // 0.5));
    // new JoystickButton(driver, GREEN_BUTTON).onTrue(new TimedTestWheelTurn(D,
    // 5000));

    // new JoystickButton(driver, GREEN_BUTTON)
    // .onTrue(new SequentialCommandGroup(
    // new ResetLocationCommand(D, Pose2d.kZero),
    // new WaitCommand(5),
    // new DriveToLocation(D, lc,
    // new PathContainer().addWaypoint(new Pose2d(2.196, 1.994,
    // Pose2d.kZero.getRotation()))
    // ),
    // new FaceTowardsCoordinates(D,
    // 11.914,
    // 4.051,
    // () -> 0,
    // () -> 0)
    // ));

    C.setDefaultCommand(new ClimbTelop(C, () -> operator.getRawButton(14), () -> operator.getRawButton(8)));

    I.setDefaultCommand(
        new ExtendOrRectactIntake(I, () -> operator.getRawButton(12), () -> operator.getRawButton(13),
            () -> operator.getRawButton(20)));

    // new JoystickButton(testing, RED_BUTTON)
    // .onTrue(new RetractIntake(I));

    new JoystickButton(operator, 18)
        .whileTrue(new ReverseShoot(S, A));

    new JoystickButton(operator, 19)
        .whileTrue(new TelopLadderShoot(S, A));

    new JoystickButton(operator, 11)
        .whileTrue(new Testing_Shoot(S, b, A, D::getPose));

    new JoystickButton(operator, 17)
        .whileTrue(new TelopTrenchShoot(S, A));

    new JoystickButton(operator, 10)
        .whileTrue(new TelopAlleniceWallShot(S, A));

    // new JoystickButton(operator, RED_BUTTON)
    // .whileTrue(new Advance(A));

    // new JoystickButton(operator, 10)
    // .whileTrue(new ShootFromDepot(S, A, b));

    new JoystickButton(driver, GREEN_BUTTON)
        .whileTrue(new XTheWheels(D));

    // new JoystickButton(operator, 10)
    // .whileTrue(new ShootIntake(S, A, I));

    // new JoystickButton(driver, BLUE_BUTTON)
    // .onTrue(new Spin180(D));

    System.out.println("Ended configureBindings()");
  }

  public void resetGyroForAuto() {
    Command selected = PathplannerautoChoosers.getSelected();
    // if (selected instanceof PathPlannerAuto) {
    // Pose2d startingPose = ((PathPlannerAuto) selected).getStartingPose();
    // if (startingPose != null) {
    // D.resetGyroToAngle(startingPose.getRotation().getDegrees());
    // }

  }

  public void teleopPeriodic() {
    // C.JoystickControl(climber.getRawAxis(LEFT_Y_AXIS));
    // I.testSliders(operator.getRawAxis(LEFT_X_AXIS));
  }

  public void autonomousExit() {
    autoTimer.stop();
  }

  public void Periodic() {
    MetricService.publish(MetricName.AUTO_TIMER, autoTimer.get());
    note_entry.getString("");
    updateVisionEst();
    Pose2d poseEstimate = D.getPose();
    overallPoseEstimate.setRobotPose(poseEstimate);
    MetricService.publishRobotLocation(poseEstimate);
    MetricService.periodic();

    DIO_entry.setBoolean(b.getHopper());
    DIO_entry1.setBoolean(b.getShooter());
    DIO_entry2.setBoolean(limitSwitches.getClimber());
    DIO_entry3.setBoolean(limitSwitches.getLimit2());
  }

  private void updateVisionEst() {
    if (V == null)
      return;
    if (DriverStation.isAutonomous())
      return;
    V.getEstimatedVisionPoses().forEach(estimateContainer -> {
      D.addVisionMeasurement(estimateContainer.estimatedPose().estimatedPose.toPose2d(),
          estimateContainer.estimatedPose().timestampSeconds, estimateContainer.stdDev());
      visionPoseEstimate.setRobotPose(estimateContainer.estimatedPose().estimatedPose.toPose2d());
    });
  }

  private double getDistanceToHubMeters() {
    Pose2d pose = D.getPose();
    double deltaX = HUB_X_METERS - pose.getX();
    double deltaY = HUB_Y_METERS - pose.getY();
    return Math.hypot(deltaX, deltaY);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new WaitCommand(1000);
    autoTimer.reset();
    autoTimer.start();
    MetricService.publish(MetricName.AUTO_TIMER, autoTimer.get());
    return PathplannerautoChoosers.getSelected();
  }
}
