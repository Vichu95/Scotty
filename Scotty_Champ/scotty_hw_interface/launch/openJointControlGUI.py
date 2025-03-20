#!/usr/bin/env python
from std_msgs.msg import String
import webbrowser
import rospkg

## Open the GUI
rospack = rospkg.RosPack()
package_path = rospack.get_path("scotty_hw_interface")
webbrowser.open(package_path + "/scripts/calibrator.html")