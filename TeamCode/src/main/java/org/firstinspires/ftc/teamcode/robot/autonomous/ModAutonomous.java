package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utilities.RobotHardware;

@Autonomous(name = "Mod Autonomous", group = "Testing")
public class ModAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.resetEncoders();

        waitForStart();

        robot.stageOne.setPower(-0.7);

        robot.move(600,0,0.7);

        while (robot.getHeading() < 45) {
            robot.tankLeft(0.35);
        }
        robot.stopMotors();

        robot.stageOne.setPower(0);

        wait(1000);

        /*

        while (robot.getHeading() > -85) {
            robot.tankRight(0.35);
        }
        robot.stopMotors();

        robot.stageOne.setPower(-1);

        while (robot.colorSensor.blue() < 500) {
            robot.setPower(0.35,0.35,0.35,0.35);
        }
        robot.stopMotors();
        ?
         */

        stop();
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
