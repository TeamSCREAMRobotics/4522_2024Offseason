// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc2025.constants.Constants;
import frc2025.dashboard.Dashboard.DashboardValue;
import frc2025.logging.Logger;
import frc2025.logging.NoteVisualizer;
import frc2025.subsystems.vision.Vision;
import util.RunnableUtil.RunOnce;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private RunOnce autoConfigurator = new RunOnce();

  private static boolean isSim = TimedRobot.isSimulation();

  public static boolean isSimulation() {
    return isSim;
  }

  public Robot() {
    DataLogManager.start();

    if (isSimulation()) {
      addPeriodic(
          RobotContainer.getSubsystems().drivetrain()::updateSimState, Constants.SIM_PERIOD_SEC);
    }
  }

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    switch (Constants.ROBOT_MODE) {
      case REAL:
        Logger.setPdh(new PowerDistribution());
        Logger.setOptions(
            new DogLogOptions()
                .withCaptureDs(true)
                .withCaptureNt(true)
                .withLogExtras(true)
                .withNtPublish(false));
        // SignalLogger.setPath("/media/sda1/");
        // SignalLogger.start();
        break;

      case SIM:
        Logger.setOptions(
            new DogLogOptions()
                .withCaptureDs(true)
                .withCaptureNt(true)
                .withLogExtras(false)
                .withNtPublish(true));
        break;
    }

    Logger.setEnabled(true);
    CommandScheduler.getInstance().onCommandInitialize(RobotState::addActiveCommand);
    CommandScheduler.getInstance().onCommandFinish(RobotState::removeActiveCommand);
    CommandScheduler.getInstance().onCommandInterrupt(RobotState::removeActiveCommand);

    FollowPathCommand.warmupCommand().schedule();
  }

  DashboardValue noteReset = new DashboardValue("Reset Notes", false);

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotState.outputTelemetry();
    Vision.periodic();

    /* autoConfigurator.runOnceWhen(
    () -> {
      System.out.println("[Init] Ready to Enable!");
    },
    DriverStation.getAlliance().isPresent()); */

    if (isSimulation() && noteReset.get()) {
      NoteVisualizer.resetNotes();
      noteReset.set(false);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    NoteVisualizer.resetNotes();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
