package frc.robot;

import edu.wpi.first.math.util.Units;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    // Photon camera for AprilTags
    private PhotonCamera aprilTagCam = new PhotonCamera("AprilTag");
    // Photon camera for notes
    private PhotonCamera noteCamera = new PhotonCamera("Notes");

    // Saved results from last refresh for AprilTag camera
    private PhotonPipelineResult aprilTagResult;
    // Saved results from last refresh for note camera
    private PhotonPipelineResult notesCamResult;

    public Vision() {
        // Refresh the latest result on initialization
        refresh();
    }

    // Call this function to update the camera results to the latest
    public void refresh() {
        // Store getLatestResult in the result variables for later use
        this.aprilTagResult = aprilTagCam.getLatestResult();
        this.notesCamResult = noteCamera.getLatestResult();
    }

    // Return whether the AprilTag camera has a target
    public boolean hasAprilTag() {
        return aprilTagResult.hasTargets();
    }

    // Return an AprilTag with the given ID found by the AprilTag camera
    // Will return null if the given AprilTag is not found
    public PhotonTrackedTarget getAprilTag(int id) {
        // Return null if no AprilTag is found
        if (!hasAprilTag()) {
            return null;
        }

        // Iterate through all the AprilTags found
        for (PhotonTrackedTarget aprilTag : aprilTagResult.getTargets()) {
            // If an AprilTag with a matching ID is found, return it
            if (aprilTag.getFiducialId() == id) {
                return aprilTag;
            }
        }

        // If none of the AprilTags found match, return null
        return null;
    }

    // Return the horizontal distance away from the given AprilTag
    public double getAprilTagDistance(PhotonTrackedTarget aprilTag) {
        // Calculate distance based on camera height/angle and AprilTag height/angle
        double baseDistance = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(15.318), 
            Units.inchesToMeters(56.88), 
            Units.degreesToRadians(15.0), 
            Units.degreesToRadians(aprilTag.getPitch()));
        
        // Use this magic formula to convert calculated distance to actual distance (close enough)
        return (baseDistance * 0.844) + 0.303;
    }

    // Return the angle the wrist needs to be at to shoot from the given distance
    public double getShootingAngle(double distance) {
        // Use this magic formula to convert distance to shooting angle
        // Hopefully we never have to change this :)
        return (distance * 0.9) - 11.1; // distance * 0.9 - 11.1
    }

    // Return whether the note camera has a target
    public boolean hasNote() {
        return notesCamResult.hasTargets();
    }

    // Return the largest target found by the note camera
    // Will return null if no targets are found
    public PhotonTrackedTarget bestNote() {
        // If no notes are found, return null
        if (!hasNote()) {
            return null;
        }

        // Return the best target found by the note camera
        return this.notesCamResult.getBestTarget();
    }
}
