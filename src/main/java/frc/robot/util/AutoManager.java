package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoCommand;

public class AutoManager {
    private SendableChooser<AutoCommand> autoChooser;

    public AutoManager() {
        List<String> ppAutoNames = AutoBuilder.getAllAutoNames();
        autoChooser = new SendableChooser<AutoCommand>();

        ppAutoNames.forEach(name -> {
            AutoCommand autoCommand = AutoCommand.fromPathPlannerAuto(name);
            addAuto(name, autoCommand);
        });

        autoChooser.setDefaultOption("No Auto", null);

        SmartDashboard.putData("Autonomous Route", autoChooser);
    }

    public AutoManager(AutoCommand[] autos) {
        this();
        for (AutoCommand auto : autos) {
            addAuto(auto);
        }
    }

    public void addAuto(AutoCommand autoCommand) {
        addAuto(autoCommand.getName(), autoCommand);
    }

    public void addAuto(String name, AutoCommand autoCommand) {
        autoChooser.addOption(name, autoCommand);
    }

    public AutoCommand getSelectedAuto() {
        return autoChooser.getSelected();
    }

    public SendableChooser<AutoCommand> getAutoChooser() {
        return autoChooser;
    }
}
