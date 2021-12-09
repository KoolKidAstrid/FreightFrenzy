package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto Blue", group="Linear Opmode")
public class AutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();


        nyx.drive(-12, 0.5);
        nyx.setArm(-280);
        nyx.turn(-90, 0.5);
        nyx.drive(-20, 0.5);
        nyx.turn(-75, 0.5);
        nyx.drive(-8, 0.5);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(-0.025);
        }
        nyx.autoDucks(4, -0.35);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(0);
        }
        nyx.turn(-25, 0.5);
        nyx.drive(18, 0.5);

        nyx.setArm(0);
        
    }
}
