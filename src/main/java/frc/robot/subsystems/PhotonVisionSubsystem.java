package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class PhotonVisionSubsystem extends SubsystemBase {
    // Change these to match the name of the cameras in the PhotonVision UI
    static PhotonCamera Cameron = new PhotonCamera("Cameron"); // SHOOTER CAM
    static PhotonCamera Camille = new PhotonCamera("Camille"); // INTAKE CAM

    public static double getDistanceToGoalMeters(double odometryDistance) {
        double DisToTargetMeters = odometryDistance; // Sets a default value when to targets are seen
        var resultCameron = Cameron.getLatestResult();
        if (resultCameron.hasTargets()) {
            // Distance to target calculation
            double cameraHeightMeters = .66; // the actual height
            double targetHeightMeters = 2.6035; // the actual height
            double cameraPitchRadians = Units.degreesToRadians(20); // WILL CHANGE
            double targetPitchRadians = Units.degreesToRadians(resultCameron.getBestTarget().getPitch());
            DisToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters,cameraPitchRadians, targetPitchRadians);
            SmartDashboard.putNumber("Dis To Target", DisToTargetMeters);
        }
        return DisToTargetMeters;
    }

    public double getXDisplacementOfGoal() {
        var resultsCameron = Cameron.getLatestResult();
        double XDisplacementOfGoal = resultsCameron.getBestTarget().getYaw();
        return XDisplacementOfGoal;
    }

    public void setAlliancePipelinesRed() {
        Camille.setPipelineIndex(1); // Pipeline that tracks red balls
    }

    public void setAlliancePipelinesBlue() {
        Camille.setPipelineIndex(2); // Pipeline that tracks blue balls
    }

    public double getDistanceToBallMeters() {
        double DisToTargetMeters = 0; // Sets a default value when to targets are seen
        var resultCamille = Camille.getLatestResult();
        if (resultCamille.hasTargets()) {
            // Distance to target calculation
            double cameraHeightMeters = .9; // WILL CHANGE ON REAL ROBOT
            double targetHeightMeters = 0.15; // MIGHT CHANGE
            double cameraPitchRadians = Units.degreesToRadians(20); // WILL CHANGE ON REAL ROBOT
            double targetPitchRadians = Units.degreesToRadians(resultCamille.getBestTarget().getPitch()); // WILL CHANGE ON REAL ROBOT
            DisToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters,cameraPitchRadians, targetPitchRadians);
            SmartDashboard.putNumber("Dis To Ball", DisToTargetMeters);
        }
        return DisToTargetMeters;
    }
}