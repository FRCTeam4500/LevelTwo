package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;

public record SysIDCommands(
    Command dynamicForward,
    Command dynamicReverse,
    Command quasistaticForward,
    Command quasistaticReverse
) {}
