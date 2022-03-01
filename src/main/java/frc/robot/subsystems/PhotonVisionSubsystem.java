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

    @Override
    public void periodic() {
        
        getDistanceToGoalMeters(0);
        //getXDisplacementOfGoal();
        //getDistanceToBallMeters();

    }
    public static double getDistanceToGoalMeters(double odometryDistance) {
        double groundDisToTarget;
        var resultCameron = Cameron.getLatestResult(); // Gets the camera's results
        if (resultCameron.hasTargets()) {
            // Distance to target calculation
            double cameraHeightMeters = Units.inchesToMeters(28.75); // TODO
            double targetHeightMeters = 2.6035; // the actual height
            double cameraPitchRadians = Units.degreesToRadians(55); // TODO
            double targetPitchRadians = Units.degreesToRadians(resultCameron.getBestTarget().getPitch());
            double DisToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters,targetHeightMeters, cameraPitchRadians, targetPitchRadians);
            groundDisToTarget = DisToTargetMeters * Math.cos(resultCameron.getBestTarget().getPitch());
            SmartDashboard.putNumber("Ground Distance To Target", groundDisToTarget); // Puts the distance to SmartDashboard
        } else {
            groundDisToTarget = odometryDistance; // Sets a default value when no targets are seen
        }
        return groundDisToTarget; // Returns the distance to the goal
    }

    public double getXDisplacementOfGoal() {
        var resultsCameron = Cameron.getLatestResult();
        double XDisplacementOfGoal = 0;
        if (resultsCameron.hasTargets()) {
            XDisplacementOfGoal = resultsCameron.getBestTarget().getYaw();
        } else {
            XDisplacementOfGoal = 0;
        }
        return XDisplacementOfGoal; // Returns the X displacement from the center of the camera's view to the goal
    }

    public double getYDisplacementOfGoal() {
        var resultsCameron = Cameron.getLatestResult();
        
        double YDisplacementOfGoal = 0;
        if (resultsCameron.hasTargets()) {
            YDisplacementOfGoal = resultsCameron.getBestTarget().getPitch();
        } else {
            YDisplacementOfGoal = 0;
        }
        return YDisplacementOfGoal; // Returns the X displacement from the center of the camera's view to the goal
    }

    public void setAlliancePipelinesRed() {
        Camille.setPipelineIndex(1); // Pipeline that tracks red balls
    }

    public void setAlliancePipelinesBlue() {
        Camille.setPipelineIndex(2); // Pipeline that tracks blue balls
    }

    public double getDistanceToBallMeters() {
        double DisToTargetMeters = 0; // Sets a default value when no targets are seen
        var resultCamille = Camille.getLatestResult(); // Gets the camera's results
        if (resultCamille.hasTargets()) {
            // Distance to target calculation
            double cameraHeightMeters = Units.inchesToMeters(28.75); // TODO
            double targetHeightMeters = 0.15; // TODO
            double cameraPitchRadians = Units.degreesToRadians(20); // TODO
            double targetPitchRadians = Units.degreesToRadians(resultCamille.getBestTarget().getPitch());
            DisToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters,
                    cameraPitchRadians, targetPitchRadians);
            SmartDashboard.putNumber("Dis To Ball", DisToTargetMeters); // Puts the distance to SmartDashboard
        }
        return DisToTargetMeters; // Returns the distance to the best target ball
    }
}