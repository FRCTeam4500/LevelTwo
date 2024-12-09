// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.GamePieceManager;
import frc.robot.utilities.logging.HoundLog;

public class Robot extends TimedRobot {
    private DogLogOptions homeOptions = new DogLogOptions(true, true, true, true, 1000);
    private DogLogOptions compOptions = new DogLogOptions(false, true, true, true, 1000);
    private Superstructure structure = new Superstructure();
    private CommandXboxController xbox = new CommandXboxController(2);
    
    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);

        setupLogging();
        setupController();
    }

    public void setupController() {
        xbox.leftTrigger()
            .onTrue(structure.startIntake())
            .onFalse(structure.finishIntake());
        
        xbox.rightTrigger()
            .onTrue(structure.readyAmp())
            .onFalse(structure.fire());
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
