package frc.lib.NinjasLib.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;

/**
 * Wrapper around PathPlanner's {@link AutoBuilder} that only exposes auto-building and
 * auto-choosing functionality. Unlike AutoBuilder, {@link #buildAuto} lets you choose whether the
 * auto should be mirrored and returns a {@link PathPlannerAuto} (so you get access to info like
 * the starting pose), and the choosers return the auto's name rather than a built command/auto -
 * pass the chosen name to {@link #buildAuto} yourself to actually build it with your desired
 * mirror setting.
 */
public class NinjasAutoBuilder {
    /**
     * Configures the underlying AutoBuilder for using PathPlanner's built-in commands.
     *
     * @param poseSupplier a supplier for the robot's current pose
     * @param resetPose a consumer for resetting the robot's pose
     * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
     *     speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor.
     * @param controller Path following controller that will be used to follow paths
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side
     *     of the field.
     * @param driveRequirements the subsystem requirements for the robot's drive train
     */
    public static void configure(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        Subsystem... driveRequirements) {
        AutoBuilder.configure(
            poseSupplier,
            resetPose,
            robotRelativeSpeedsSupplier,
            output,
            controller,
            robotConfig,
            shouldFlipPath,
            driveRequirements);
    }

    /**
     * Builds a {@link PathPlannerAuto} for the given auto name.
     *
     * @param autoName the name of the auto to build
     * @param mirror whether to mirror the auto to the other side of the current alliance
     * @return the built auto
     */
    public static PathPlannerAuto buildAuto(String autoName, boolean mirror) {
        return new PathPlannerAuto(autoName, mirror);
    }

    /**
     * Create and populate a sendable chooser with the names of all PathPlannerAutos in the
     * project. The default option is an empty string.
     *
     * @return SendableChooser populated with all auto names
     */
    public static SendableChooser<String> buildAutoChooser() {
        return buildAutoChooser("");
    }

    /**
     * Create and populate a sendable chooser with the names of all PathPlannerAutos in the
     * project.
     *
     * @param defaultAutoName The name of the auto that should be the default option. If this is
     *     an empty string, or if an auto with the given name does not exist, the default option
     *     will be "None" (empty string).
     * @return SendableChooser populated with all auto names
     */
    public static SendableChooser<String> buildAutoChooser(String defaultAutoName) {
        return buildAutoChooserWithOptionsModifier(defaultAutoName, (stream) -> stream);
    }

    /**
     * Create and populate a sendable chooser with the names of all PathPlannerAutos in the
     * project. The default option is an empty string.
     *
     * @param optionsModifier A lambda function that can be used to modify the auto names before
     *     they go into the chooser
     * @return SendableChooser populated with all auto names
     */
    public static SendableChooser<String> buildAutoChooserWithOptionsModifier(
        Function<Stream<String>, Stream<String>> optionsModifier) {
        return buildAutoChooserWithOptionsModifier("", optionsModifier);
    }

    /**
     * Create and populate a sendable chooser with the names of all PathPlannerAutos in the
     * project.
     *
     * @param defaultAutoName The name of the auto that should be the default option. If this is
     *     an empty string, or if an auto with the given name does not exist, the default option
     *     will be "None" (empty string).
     * @param optionsModifier A lambda function that can be used to modify the auto names before
     *     they go into the chooser
     * @return SendableChooser populated with all auto names
     */
    public static SendableChooser<String> buildAutoChooserWithOptionsModifier(
        String defaultAutoName, Function<Stream<String>, Stream<String>> optionsModifier) {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                "NinjasAutoBuilder was not configured before attempting to build an auto chooser");
        }

        SendableChooser<String> chooser = new SendableChooser<>();
        List<String> autoNames = AutoBuilder.getAllAutoNames();

        boolean hasDefault = !defaultAutoName.isEmpty() && autoNames.contains(defaultAutoName);

        if (hasDefault) {
            chooser.setDefaultOption(defaultAutoName, defaultAutoName);
            chooser.addOption("None", "");
        } else {
            chooser.setDefaultOption("None", "");
        }

        optionsModifier
            .apply(autoNames.stream().filter(name -> !hasDefault || !name.equals(defaultAutoName)))
            .forEach(name -> chooser.addOption(name, name));

        return chooser;
    }
}