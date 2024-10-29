package frc.robot.libraries.external.robot.input;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * <p>An interface for easily implementing gamepads as an input source.</p>
 *
 * @author Jacob Bublitz
 * @see XboxController
 * @since 1.0
 */
public abstract class Controller {
	public abstract Axis getLeftTriggerAxis();

	public abstract Axis getLeftXAxis();

	public abstract Axis getLeftYAxis();

	public abstract Axis getRightTriggerAxis();

	public abstract Axis getRightXAxis();

	public abstract Axis getRightYAxis();

	/**
	 * Get the A Trigger of the controller.
	 *
	 * @return The A button
	 * @since 1.0
	 */
	public abstract Trigger getAButton();

	/**
	 * Get the B Trigger of the controller.
	 *
	 * @return The B button
	 * @since 1.0
	 */
	public abstract Trigger getBButton();

	/**
	 * Get the X Trigger of the controller.
	 *
	 * @return The X button
	 * @since 1.0
	 */
	public abstract Trigger getXButton();

	/**
	 * Get the Y Trigger of the controller.
	 *
	 * @return The Y button
	 * @since 1.0
	 */
	public abstract Trigger getYButton();

	/**
	 * Get the left bumper Trigger of the controller.
	 *
	 * @return The left bumper button
	 * @since 1.0
	 */
	public abstract Trigger getLeftBumperButton();

	/**
	 * Get the right bumper Trigger of the controller.
	 *
	 * @return The right bumper button
	 * @since 1.0
	 */
	public abstract Trigger getRightBumperButton();

	/**
	 * Get the back Trigger of the controller.
	 *
	 * @return The back button
	 * @since 1.0
	 */
	public abstract Trigger getBackButton();

	/**
	 * Get the start Trigger of the controller.
	 *
	 * @return The start button
	 * @since 1.0
	 */
	public abstract Trigger getStartButton();

	/**
	 * Get the left joystick Trigger of the controller.
	 *
	 * @return The left joystick button
	 * @since 1.0
	 */
	public abstract Trigger getLeftJoystickButton();

	/**
	 * Get the right joystick Trigger of the controller.
	 *
	 * @return The right joystick button
	 * @since 1.0
	 */
	public abstract Trigger getRightJoystickButton();

	/**
	 * Get a D-Pad Trigger of the controller.
	 *
	 * @param direction The direction of the D-Pad button
	 * @return The D-Pad Trigger of the specified direction
	 * @since 1.0
	 */
	public abstract Trigger getDPadButton(DPadButton.Direction direction);
}
