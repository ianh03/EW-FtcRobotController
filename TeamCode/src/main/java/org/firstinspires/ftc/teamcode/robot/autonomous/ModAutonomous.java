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

        robot.move(695,0,0.7);

        while (robot.getHeading() < 50) {
            robot.tankLeft(0.35);
        }
        robot.stopMotors();

        wait(500);

        while (robot.getHeading() > -83) {
            robot.tankRight(0.35);
        }
        robot.stopMotors();

        while (robot.colorSensor.blue() < 500) {
            robot.setPower(0.4,0.4,0.4,0.4);
        }
        robot.stopMotors();

        robot.move(210, -83, 0.7);

        wait(500);

        robot.reverse(300, -85,-0.5);

        while (robot.getHeading() > -83) {
            robot.tankRight(0.35);
        }
        robot.stopMotors();


        /*wait(750);

        robot.move(275, -83, -0.7);
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
