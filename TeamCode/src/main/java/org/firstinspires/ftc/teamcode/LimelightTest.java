package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="Furries are awesome UwU")

public class LimelightTest extends LinearOpMode{

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());

                    List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
                    for (int i = 0; i < aprilTags.toArray().length; i++) {
                        telemetry.addData("Fiducial ID's", aprilTags.get(i).getFiducialId());
                        telemetry.addData("AprilTag Degrees", aprilTags.get(i).getTargetXDegrees());
                        telemetry.addData("Target Vector", aprilTags.get(i).getCameraPoseTargetSpace().toString());
                    }

                    telemetry.update();
                }
            }
        }
    }
}
