package org.ironriders.lib;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {
    String m_name;
    Transform3d m_offset;
    Double m_trustWeight;

    PhotonCamera m_photonCamera;
    PhotonPoseEstimator m_estimator;

    PhotonPipelineResult m_mostRecent;

    public List<PhotonTrackedTarget> m_targets;

    private AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public PhotonCamera getPhotonCamera() {
        return m_photonCamera;
    }

    public PhotonPoseEstimator getEstimator() {
        return m_estimator;
    }

    public String getName() {
        return m_name;
    }

    public Double getWeight() {
        return m_trustWeight;
    }

    public Transform3d getOffset() {
        return m_offset;
    }

    /*
     * You should call this every tick exactly once. (Will probably be fine if
     * called more often)
     */
    public void updateResultBuffer() {
        List<PhotonPipelineResult> results = m_photonCamera.getAllUnreadResults();

        if (results == null || results.size() <= 0) {
            return; // don't update the buffer if we get a null response. Could be incorrect.
        }

        m_mostRecent = results.get(results.size() - 1); // get the most recent

        m_targets = m_mostRecent.targets;
    }

    /*
     * Get the most recent result from this camera.
     * Could potentially return null.
     */
    public PhotonPipelineResult getResult() {
        if (m_mostRecent == null) {
            updateResultBuffer();
        }

        if (m_mostRecent == null) { // could still be null after the update, check again
            m_mostRecent = new PhotonPipelineResult();
        }

        return m_mostRecent;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return m_targets;
    }

    public boolean seesTargets() {
        return getResult().hasTargets();
    }

    /*
     * Define a new camera.
     * 
     * @param name is the camera name set in the photonvision dashboard.
     * 
     * @param offset is the offset from the center of the robot, positive x towards
     * the battery.
     * 
     * @param trustWeight is the weight on the trust we have in estimations made
     * by this camera. Useful if you have one crapy camera and one good one or
     * something similar. Should be in the range [-1 (least trusting) to 1 (most
     * trusting)]. Zero is no weighing.
     */
    public VisionCamera(String name, Transform3d offset, Double trustWeight) {
        m_name = name;
        m_offset = offset;
        m_trustWeight = Utils.clamp(-1, 1, trustWeight);

        m_photonCamera = new PhotonCamera(name);
        m_estimator = new PhotonPoseEstimator(m_fieldLayout, offset);
    }

    /*
     * Define a new camera.
     * 
     * @param name is the camera name set in the photonvision dashboard.
     * 
     * @param offset is the offset from the center of the robot, positive x towards
     * the battery.
     */
    public VisionCamera(String name, Transform3d offset) {
        m_name = name;
        m_offset = offset;
        m_trustWeight = 0d;

        m_photonCamera = new PhotonCamera(name);
        m_estimator = new PhotonPoseEstimator(m_fieldLayout, offset);
    }
}
