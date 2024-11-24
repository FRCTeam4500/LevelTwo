package frc.robot.utilities;

import java.util.Set;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utilities.logging.HoundLog;

public class GamePieceManager {
    private static HashMap<NetworkTable, Pose3d> cameras = new HashMap<>();
    private static Set<Translation2d> pieces = new HashSet<>();
    public static void resetField() {
        pieces.clear();
        pieces.add(new Translation2d(2.9, 7));
        pieces.add(new Translation2d(2.9, 5.55));
        pieces.add(new Translation2d(2.9, 4.1));
        pieces.add(new Translation2d(8.3, 7.44));
        pieces.add(new Translation2d(8.3, 5.78));
        pieces.add(new Translation2d(8.3, 4.11));
        pieces.add(new Translation2d(8.3, 2.44));
        pieces.add(new Translation2d(8.3, 0.77));
        pieces.add(new Translation2d(13.67, 7));
        pieces.add(new Translation2d(13.67, 5.55));
        pieces.add(new Translation2d(13.67, 4.1));
        log();
    }

    public static void addPiece(Translation2d translation) {
        pieces.add(translation);
        log();
    }

    public static void removePiece(Translation2d translation) {
        pieces.remove(translation);
        log();
    }

    public static void addCamera(String name, Pose3d offset) {
        cameras.put(NetworkTableInstance.getDefault().getTable(name), offset);
    }

    private static void log() {
        Translation2d[] array = new Translation2d[pieces.size()];
        int i = 0;
        for (Translation2d piece : pieces) {
            array[i] = piece;
            i++;
        }
        HoundLog.log("Pieces", array);
    }
    
    public static void updateNT(Pose2d robotPose) {
        for (Map.Entry<NetworkTable, Pose3d> cameraEntry: cameras.entrySet()) {
            Pose3d camera = new Pose3d(robotPose).plus(new Transform3d(new Pose3d(), cameraEntry.getValue()));
            boolean seenPiece = false;
            double upAngle = 0;
            double sideAngle = 0;
            for (Translation2d piece : pieces) {
                Pose3d poseVer = new Pose3d(new Pose2d(piece, new Rotation2d()));
                Pose3d thisPiece = poseVer.relativeTo(camera);
                double thisDist = thisPiece.getTranslation().getNorm();
                double thisUp = thisPiece.getZ();
                double thisSide = thisPiece.getY();
                double thisUpAngle = Math.toDegrees(Math.asin(thisUp/thisDist));
                double thisSideAngle = Math.toDegrees(Math.asin(thisSide/thisDist));
                if (seenPiece) {
                    double thisCenterOffset = Math.hypot(thisSideAngle, thisUpAngle);
                    double seenCenterOffset = Math.hypot(sideAngle, upAngle);
                    if (thisCenterOffset < seenCenterOffset) {
                        upAngle = thisUpAngle;
                        sideAngle = thisSideAngle;
                    }
                } else {
                    if (Math.abs(thisUpAngle) < 25 && Math.abs(thisSideAngle) < 30) {
                        seenPiece = true;
                        upAngle = thisUpAngle;
                        sideAngle = thisSideAngle;
                    }
                }
            }
            NetworkTable table = cameraEntry.getKey(); 
            if (seenPiece) {
                table.getEntry("tv").setInteger(1);
                table.getEntry("tx").setNumber(-sideAngle);
                table.getEntry("ty").setNumber(upAngle);
            } else {
                table.getEntry("tv").setInteger(0);
                table.getEntry("tx").setNumber(0);
                table.getEntry("ty").setNumber(0);
            }
            
        }
    }
}
