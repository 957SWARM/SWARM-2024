package com.team957.comp2024.subsystems;

import com.team957.comp2024.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import org.littletonrobotics.Alert;
import org.littletonrobotics.Alert.AlertType;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Vision implements Subsystem {
    private final PhotonCamera camOne = new PhotonCamera("limelight"); // todo change

    private final Alert camOneDisconnected =
            new Alert("PhotonCamera camOne disconnected!", AlertType.ERROR);

    // PhotonCamera camTwo = new PhotonCamera...

    private final AprilTagFieldLayout tags =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final PhotonPoseEstimator camOneEstimator =
            new PhotonPoseEstimator(
                    tags,
                    Constants.VisionConstants.POSE_STRATEGY,
                    camOne,
                    Constants.VisionConstants.CAM_ONE_TO_ROBOT_TRANSFORM);

    private final Consumer<EstimatedRobotPose> localizationInput;

    public Vision(Consumer<EstimatedRobotPose> localizationInput) {
        camOneEstimator.setMultiTagFallbackStrategy(
                Constants.VisionConstants.POSE_FALLBACK_STRATEGY);

        this.localizationInput = localizationInput;

        register();
    }

    @Override
    public void periodic() {
        var camOneEstimate = camOneEstimator.update();

        if (camOneEstimate.isPresent()) {
            localizationInput.accept(camOneEstimate.get());
        }

        camOneDisconnected.set(camOne.isConnected());
    }
}
