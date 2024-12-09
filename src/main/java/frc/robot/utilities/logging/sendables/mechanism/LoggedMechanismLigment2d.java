package frc.robot.utilities.logging.sendables.mechanism;

import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

import java.util.HashMap;

public class LoggedMechanismLigment2d implements Loggable {
    private double angle;
    private double length;
    private HashMap<String, LoggedMechanismLigment2d> ligments;

    public LoggedMechanismLigment2d(double length, double angle) {
        this.angle = angle;
        this.length = length;
        ligments  = new HashMap<>();
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public void setLength(double length) {
        this.length = length;
    }
    
    public void append(String name, LoggedMechanismLigment2d ligment) {
        ligments.put(name, ligment);
    }

    @Override
    public void log(String name) {
        HoundLog.log(name + "/.type", "line");
        HoundLog.log(name + "/angle", angle);
        HoundLog.log(name + "/color", "#EB8934");
        HoundLog.log(name + "/length", length);
        HoundLog.log(name + "/weight", 10);
        for (String ligmentName : ligments.keySet()) {
            ligments.get(ligmentName).log(name + "/" + ligmentName);
        }

    }

}
