package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;

public class Robot {
    GamepadEx gp;

    shooterSystem s;
    intakeSystem i;
    drivetrainSystem d;
    liftSystem l;
    RobotConstants.robotState currentState;

    public shooterSystem getS() {
        return s;
    }

    public intakeSystem getI() {
        return i;
    }

    public drivetrainSystem getD() {
        return d;
    }

    public liftSystem getL() {
        return l;
    }

    public Robot(final HardwareMap hmap) {
        s = new shooterSystem(hmap);
        i = new intakeSystem(hmap);
        d = new drivetrainSystem(hmap);
        l = new liftSystem(hmap);
        currentState = RobotConstants.robotState.IDLE;
        for (LynxModule mod : hmap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void periodic() {
        i.periodic();
        s.periodic();
        d.periodic();
        l.periodic();
    }

    public void setShooterValues() {
        s.setSpeed(d.yoCalcSpeed());
        s.setHoodPosition(d.yoCalcHood());
        s.setTurretPosition(d.yoCalcAim());
    }

}
