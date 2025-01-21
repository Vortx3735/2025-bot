package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private PhotonPipelineResult latestResult;

  public VisionSubsystem(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }

  @Override
  public void periodic() {
    // Update vision data periodically
    latestResult = camera.getLatestResult();
  }

  public boolean isTargetDetected() {
    return latestResult.hasTargets();
  }

  public double getTargetYaw() {
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget().getYaw();
    }
    return 0.0;
  }

  public double getTargetDistance() {
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget().getArea(); // Replace with your distance logic
    }
    return 0.0;
  }
}
