package frc.robot.subsystems;

import java.util.Optional;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraVision extends SubsystemBase {

    public static final double MIN_TA_BACK = 0.15;
    public static final double MIN_TA_FRONT = 0.1;

    private static InterpolatingDoubleTreeMap mapping = new InterpolatingDoubleTreeMap();

    static {
        mapping.put(Double.valueOf(14.85), Double.valueOf(-0.11));
        mapping.put(Double.valueOf(9.74), Double.valueOf(-0.119));
        mapping.put(Double.valueOf(6.31), Double.valueOf(-0.127));
        mapping.put(Double.valueOf(3.25), Double.valueOf(-0.137));
        mapping.put(Double.valueOf(0.58), Double.valueOf(-0.144));
        mapping.put(Double.valueOf(-1.66), Double.valueOf(-0.151 ));
        mapping.put(Double.valueOf(-3.09), Double.valueOf(-0.154));
        mapping.put(Double.valueOf(-4.56), Double.valueOf(-0.159));
        mapping.put(Double.valueOf(-5.95), Double.valueOf(-0.164));
        mapping.put(Double.valueOf(-7.11), Double.valueOf(-0.1655));
        mapping.put(Double.valueOf(-8.04), Double.valueOf(-0.1668));
        mapping.put(Double.valueOf(-8.62), Double.valueOf(-0.171));
    }

    public static double getUnderhandAngle() {
        return mapping.get(Double.valueOf(LimelightHelpers.getTY("limelight-front")));
    }

    public static Optional<Double> backCamAngleToGoal() {
        if (LimelightHelpers.getTA("limelight-back") >= MIN_TA_BACK) {
            return Optional.of(LimelightHelpers.getTX("limelight-back"));
        } else {
            return Optional.empty();
        }
    }

    public static Optional<Double> frontCamAngleToGoal() {
        if (LimelightHelpers.getTA("limelight-front") >= MIN_TA_FRONT) {
            return Optional.of(LimelightHelpers.getTX("limelight-front"));
        } else {
            return Optional.empty();
        }
    }
}