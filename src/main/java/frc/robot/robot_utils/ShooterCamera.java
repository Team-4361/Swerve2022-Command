package frc.robot.robot_utils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;

import static frc.robot.Constants.ShooterCameraConsts.TAPE_HEIGHT;

public class ShooterCamera {

    final PhotonCamera photonCamera;

    final double cameraHeight;
    final double cameraPitch;

    //Height in meters of the target
    final double TARGET_HEIGHT = TAPE_HEIGHT;

    /*
      Our networktablename is photonvision
      Our cameraName is RoxCam2021-4361
      NetworkTable server is turned off
    */

    /**
     * @param cameraName    name of the camera in photon vision
     * @param mCameraHeight camera height in meters
     * @param mCameraPitch  camera pitch in radians
     */
    public ShooterCamera(String cameraName, double mCameraHeight, double mCameraPitch) {
        cameraHeight = mCameraHeight;

        //camera pitch in radians
        cameraPitch = mCameraPitch;

        photonCamera = new PhotonCamera(cameraName);

        photonCamera.setPipelineIndex(0);

        System.out.println("Connected");
    }

    //Creates an image of the input video of the pipeline
    public void takeInputPicture() {
        photonCamera.takeInputSnapshot();
    }

    //Creates an image of the output video of the pipeline
    public void takeOutputSnapshot() {
        photonCamera.takeOutputSnapshot();
    }

    /**
     * @return returns the best target
     */
    public PhotonTrackedTarget getBestTarget() {
        return photonCamera.getLatestResult().getBestTarget();
    }

    //Gets the distance to the best target
    private double getDistanceToTarget(PhotonTrackedTarget target) {
        return PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, TARGET_HEIGHT, cameraPitch, Math.toRadians(target.getPitch()));
    }


    // Returns the yaw--rotation around the vertical axis--in degrees
    // 0 Yaw means the target is exactly in the middle of the screen
    // Negative yaw means the target is somewhere on the left of the screen
    // Positive yaw means the target is somewhere on the right of the screen
    private double getYaw(PhotonTrackedTarget target) {
        return target.getYaw();
    }

    private double getPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    /**
     * @return returns a Hashmap containing the distance, yaw, and pitch of the best target
     * Distance, Yaw, Pitch are they key for this information
     */
    public HashMap<String, Double> getTargetGoal() {
        HashMap<String, Double> goalInfo = new HashMap<>();
        PhotonTrackedTarget trackedTarget = getBestTarget();

        if (trackedTarget == null) {
            goalInfo.put("Distance", 0.0);
            goalInfo.put("Yaw", 0.0);
            goalInfo.put("Pitch", 0.0);
            goalInfo.put("Status", 0.0);

            return goalInfo;
        }

        goalInfo.put("Distance", getDistanceToTarget(trackedTarget));
        goalInfo.put("Yaw", getYaw(trackedTarget));
        goalInfo.put("Pitch", getPitch(trackedTarget));
        goalInfo.put("Status", 1.0);

        return goalInfo;
    }

}