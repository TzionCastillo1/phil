# PHIL
PHIL stands for Pocket Hardware In the Loop. While developing a flight controller and accompanying flight control software, I realized that there is no good way of doing HIL testing at the hobby level. This is designed specifically for avioniocs for small quadcopters, enabling the user to test flight circuitry and code without risking their hardware. For this reason, PHIL is designed to be a lightweight, and eaily integrable method of adding HIL testing to hobbyist embedded projects. To use PHIL, a model for the DUT needs to be created in the "dut_description/dut" folder.

Note: Need to disable access control for the display before running the container:
'''
xhost +
'''
 Disable after running the container.
'''
xhost -
'''