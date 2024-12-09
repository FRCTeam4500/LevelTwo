// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.GamePieceManager;
import frc.robot.utilities.logging.HoundLog;

public class Robot extends TimedRobot {
    private DogLogOptions homeOptions = new DogLogOptions(true, true, true, true, 1000);
    private DogLogOptions compOptions = new DogLogOptions(false, true, true, true, 1000);
    private Swerve swerve = new Swerve();
    private Superstructure structure = new Superstructure();
    private CommandXboxController xbox = new CommandXboxController(2);
    
    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        swerve.setDefaultCommand(swerve.angleCentric(xbox.getHID()));

        setupLogging();
        setupDriveController();
        setupAuto();
    }

    public void setupDriveController() {
        Trigger onBlue = new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
        Trigger onRed = onBlue.negate();
        Trigger faceForwards = new Trigger(() -> xbox.getRightY() < -0.5);
        Trigger faceBackwards = new Trigger(() -> xbox.getRightY() > 0.5);
        Trigger resetHeading = xbox.a();

        xbox.a().and(onBlue).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(0)));
        xbox.a().and(onRed).onTrue(swerve.resetHeading(Rotation2d.fromDegrees(180)));
        faceForwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
        faceForwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
        faceBackwards.and(onRed).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(0)));
        faceBackwards.and(onBlue).onTrue(swerve.setTargetHeading(Rotation2d.fromDegrees(180)));
        resetHeading.onTrue(swerve.resetHeading(Rotation2d.fromDegrees(0)));
    }

    public void setupAuto() {
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);
        RobotModeTriggers.autonomous().whileTrue(Commands.deferredProxy(chooser::getSelected));
    }

    public void setupLogging() {
        HoundLog.setEnabled(true);
        HoundLog.setPdh(new PowerDistribution());
        HoundLog.setOptions(homeOptions);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        Trigger atComp = new Trigger(DriverStation::isFMSAttached);
        atComp.onTrue(Commands.runOnce(() -> HoundLog.setOptions(compOptions)));
        atComp.onFalse(Commands.runOnce(() -> HoundLog.setOptions(homeOptions)));
        GamePieceManager.resetField();
    }

    @Override
    public void robotPeriodic() {
        double start = Timer.getFPGATimestamp();
        swerve.log("Swerve");
        structure.log("Superstrucutre");
        double loggingLoop = Timer.getFPGATimestamp() - start;

        start = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        double commandsLoop = Timer.getFPGATimestamp() - start;

        HoundLog.log("HoundLog/Logging Loop Time", loggingLoop * 1000);
        HoundLog.log("HoundLog/Commands Loop Time", commandsLoop * 1000);
        HoundLog.log("HoundLog/Total Loop Time", 1000 * (commandsLoop + loggingLoop));
    }
}
