package com.ninjas4744.lib.data;

public class ControllerConstants {
	/**
	 * The ID of the controller, chosen in the device's configuration software- Phoenix Tuner X / Rev
	 * Hardware Client
	 */
	public int id;

	/**
	 * Whether or not to invert the output of this controller, IF this controller is a FOLLOWER it
	 * will invert the main controller's output so if the main controller is inverted and this
	 * follower is inverted it will be inverted twice so not inverted
	 */
	public boolean inverted;
}
