package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto Red", group="Linear Opmode")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwaggyTheRobot swaggy = new SwaggyTheRobot(this);
        swaggy.initialize();
        waitForStart();

        swaggy.drive(-27, 0.5);
        swaggy.turn(90, 0.5);
        swaggy.drive(-24, 0.5);
        swaggy.turn(90, 0.5);
        swaggy.drive(-10, 0.5);
        swaggy.autoDucks(2, 0.5);


    }
}
