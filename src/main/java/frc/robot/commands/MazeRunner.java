package frc.robot.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import org.w3c.dom.events.Event;

import java.util.Optional;
import java.util.TreeMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class MazeRunner {
    private TreeMap<String, Trigger> triggers;
    private Trajectory<SwerveSample> trajectory;
    private EventLoop loop;
    private Timer timer;
    private Drivetrain drive;

    private boolean autoFlip;

    MazeRunner(String name, Drivetrain drivetrain, boolean autoMirror) throws Exception {
        Choreo.<SwerveSample>loadTrajectory(name).ifPresent((trajectory) -> this.trajectory = trajectory);
        this.drive = drivetrain;
        for(EventMarker e : this.trajectory.events()) {
            if (triggers.containsKey(e.event))
                throw new Exception("YO MR. HICE SPICE SAYS NOT TO HAVE TWO EVENTS WITH THE SAME NAME, goofus.");

            triggers.put(e.event, atTime(e.timestamp));
        }
    }

    public Trigger atTime(double timestamp) {
        return new Trigger(
                loop,
                new BooleanSupplier() {
                    double lastTimestamp = timer.get();

                    public boolean getAsBoolean() {
                        double nowTimestamp = timer.get();
                        try {
                            return lastTimestamp < nowTimestamp && nowTimestamp >= timestamp;
                        } finally {
                            lastTimestamp = nowTimestamp;
                        }
                    }
                });
    }

    public Trigger atPose(
            Supplier<Optional<Pose2d>> pose, double toleranceMeters, double toleranceRadians) {
        return new Trigger(
                loop,
                () -> {
                    Optional<Pose2d> checkedPoseOpt = pose.get();
                    return checkedPoseOpt
                            .map(
                                    (checkedPose) -> {
                                        Translation2d currentTrans = this.drive.getState().Pose.getTranslation();
                                        Rotation2d currentRot = this.drive.getState().Pose.getRotation();
                                        return currentTrans.getDistance(checkedPose.getTranslation())
                                                < toleranceMeters
                                                && Math.abs(currentRot.minus(checkedPose.getRotation()).getRadians())
                                                < toleranceRadians;
                                    })
                            .orElse(false);
                });
//                .and(active());
    }

    public Trigger atPose(Optional<Pose2d> pose, double toleranceMeters, double toleranceRadians) {
        return atPose(
                AllianceFlipUtil.optionalFlippedPose2d(pose, DriverStation::getAlliance, () -> autoFlip),
                toleranceMeters,
                toleranceRadians);
    }



    public void BindCommand(String eventName, Supplier<Command> getCommand) {

    }

//    public Command gimmeCommand() {
//        return new FunctionalCommand();
//    }
}
