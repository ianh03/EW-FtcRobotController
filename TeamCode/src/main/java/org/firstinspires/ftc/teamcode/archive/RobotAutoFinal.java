package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.utilities.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
// Autonomous Portion of our COde
@Autonomous(name = "L1Autonomous", group = "L1")
public class RobotAutoFinal extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Creating Objects of RobotHardware and Camera Classes
        RobotHardware robot = new RobotHardware(hardwareMap);
        SignalPipeline pipeline = new SignalPipeline(telemetry);

        // Setting up the Camera
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Display details before running
                robot.camera.setPipeline(pipeline);
                robot.camera.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT); //Camera Display Configuration
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode); //In case of error, we would know something went wrong
                telemetry.update(); //update display
            }
        });

        waitForStart();

        // Variable to hold the color that is detected by camera
        String selectedColor = pipeline.getAnalysis();

        // Parking Spot Number 1
        if (selectedColor.equals("BLUE")) {
            telemetry.addLine("Taking Blue Path");
            telemetry.update();

            // Move forward 345 mm while maintaining 0 heading @ 0.5 power
            robot.move(345, 0, 0.5);

            // Strafe to the Left for 310 mm
            double startingDist = robot.ticksToMM(robot.centralEncoder.getCurrentPosition());
            while (robot.ticksToMM(robot.centralEncoder.getCurrentPosition()) - startingDist < 310) {
                robot.strafeLeft(0.7);
            }
            robot.stopMotors();

            // Move forward 100 mm while maintaining 0 heading @ 0.5 power
            robot.move(100, 0, 0.5);
        }
        // Parking Spot Number 2
        else if (selectedColor.equals("GREEN")) {
            telemetry.addLine("Taking Green Path");
            telemetry.update();

            // Move forward 500 mm at 0.5 power
            robot.move(500, 0, 0.5);
        }
        // Parking Spot Number 3
        else if (selectedColor.equals("YELLOW")) {
            telemetry.addLine("Taking Yellow Path");
            telemetry.update();

            // Move forward 315 mm at 0.5 power
            robot.move(315, 0, 0.5);

            // Strafe to the Right for Parking Spot 3
            double startingDist = robot.ticksToMM(robot.centralEncoder.getCurrentPosition());
            while (robot.ticksToMM(robot.centralEncoder.getCurrentPosition()) - startingDist > -310) {
                robot.strafeRight(0.7);
            }
            robot.stopMotors();

            // Move forward 80 mm at 0.5 power
            robot.move(80, 0, 0.5);
        }
        else if (selectedColor.equals("NONE")) {
            telemetry.addLine("Unable to detect color -- going to spot 2");
            telemetry.update();

            // Move forward 500 mm at 0.5 power
            robot.move(500, 0, 0.5);
        }
        stop();
    }
}