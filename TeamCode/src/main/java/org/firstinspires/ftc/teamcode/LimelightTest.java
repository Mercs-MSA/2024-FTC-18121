package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="Limelight Test")
@Disabled
public class LimelightTest extends LinearOpMode{
    final double SAFE_DRIVE_SPEED   = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0

    final private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override
    public void runOpMode() {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true, false);

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            robot.readSensors();
            telemetry.update();
        }

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        ElapsedTime runtime = new ElapsedTime();

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();
        runtime.reset();

        boolean areThereTags = false;
        double degreesToRotate = 0;

        while (opModeIsActive()) {
            robot.readSensors();

            // Allow the driver to reset the gyro by pressing both small gamepad buttons
            if(gamepad1.start && gamepad1.back){
                robot.resetHeading();
                robot.resetOdometry();
            }

            LLResult result = limelight.getLatestResult();
            
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", robot.heading);

            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());

                    List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
                    if (aprilTags.isEmpty()) {
                        areThereTags = false;
                    } else {
                        areThereTags = true;
                        degreesToRotate = aprilTags.get(0).getTargetXDegrees();

                        telemetry.addData("Fiducial ID's", aprilTags.get(0).getFiducialId());
                        telemetry.addData("AprilTag Degrees", aprilTags.get(0).getTargetXDegrees());
                        telemetry.addData("Target Vector", aprilTags.get(0).getCameraPoseTargetSpace().toString());
                    }
                }
            }

            double drive = 0;
            double strafe = 0;
            double yaw = 0;

            if (gamepad1.a) {
                if (areThereTags) {
                    if (degreesToRotate > 0) {
                        yaw = 0.25;
                    }
                    else if (degreesToRotate < 0) {
                        yaw = -0.25;
                    }
                }
            }

            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);

            telemetry.update();
        }
    }
}
