package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class Vision {
    private PhotonCamera aprilTagCam = new PhotonCamera("AprilTag");
    private PhotonPipelineResult aprilTagResult;

    public Vision() {
        refresh();
    }

    public void refresh() {
        this.aprilTagResult = aprilTagCam.getLatestResult();
    }

    public boolean hasAprilTag() {
        return aprilTagResult.hasTargets();
    }

    public PhotonTrackedTarget getAprilTag(int id) {
        if (!hasAprilTag()) {
            return null;
        }

        for (PhotonTrackedTarget aprilTag : aprilTagResult.getTargets()) {
            if (aprilTag.getFiducialId() == id) {
                return aprilTag;
            }
        }

        return null;
    }

    public double getAprilTagDistance(PhotonTrackedTarget aprilTag) {
        return (PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(15.318), 
            Units.inchesToMeters(56.88), 
            Units.degreesToRadians(15.0), 
            Units.degreesToRadians(aprilTag.getPitch())) * 0.844) + 0.303;
    }

    public double getShootingAngle(double distance) {
        return (distance * 1.230) - 12.587;
    }
}
