package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages autonomous commands for selection on the dashboard.
 * <p>
 * The AutoManager will look for all PathPlanner paths and create an AutoCommand
 * for each one.
 * </p>
 * 
 * @see AutoCommand
 */
public class AutoManager {
    private SendableChooser<AutoCommand> autoChooser;

    /**
     * Creates a new AutoManager.
     * <p>
     * The AutoManager will look for all PathPlanner paths and create an AutoCommand
     * for each one.
     * </p>
     * 
     * @see AutoCommand
     */
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

    /**
     * Creates a new AutoManager.
     * <p>
     * The AutoManager will look for all PathPlanner paths and create an AutoCommand
     * for each one.
     * </p>
     * 
     * @param autos An array of AutoCommands to add to the AutoManager
     * @see AutoCommand
     */
    public AutoManager(AutoCommand[] autos) {
        this();
        for (AutoCommand auto : autos) {
            addAuto(auto);
        }
    }

    /**
     * Adds an AutoCommand to the AutoManager.
     * <p>
     * This will add the AutoCommand to the SendableChooser for selection on the
     * dashboard.
     * </p>
     * 
     * @param autoCommand The AutoCommand to add
     * @see AutoCommand
     */
    public void addAuto(AutoCommand autoCommand) {
        addAuto(autoCommand.getName(), autoCommand);
    }

    /**
     * Adds an AutoCommand to the AutoManager.
     * <p>
     * This will add the AutoCommand to the SendableChooser for selection on the
     * dashboard.
     * </p>
     * 
     * @param name        The name to display on the dashboard
     * @param autoCommand The AutoCommand to add
     * @see AutoCommand
     */
    public void addAuto(String name, AutoCommand autoCommand) {
        autoChooser.addOption(name, autoCommand);
    }

    /**
     * Gets the selected AutoCommand from the SendableChooser.
     * 
     * @return The selected AutoCommand, or null if none is selected
     * @see AutoCommand
     */
    public AutoCommand getSelectedAuto() {
        return autoChooser.getSelected();
    }

    /**
     * Gets the SendableChooser used by the AutoManager.
     * <p>
     * This can be used to add the chooser to a custom dashboard.
     * </p>
     * 
     * @return The SendableChooser used by the AutoManager
     * @see SendableChooser
     */
    public SendableChooser<AutoCommand> getAutoChooser() {
        return autoChooser;
    }
}
