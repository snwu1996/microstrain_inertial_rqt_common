from threading import Thread

from microstrain_inertial_msgs.srv import DeviceReport

from .common import ServiceMonitor
from .constants import _MICROSTRAIN_ROS_VERISON
from .constants import _DEFAULT_VAL

class DeviceReportMonitor(ServiceMonitor):

  def __init__(self, node, node_name):
    super(DeviceReportMonitor, self).__init__(node, node_name, "device_report", DeviceReport)

  @property
  def connected(self):
    return self._current_message is not _DEFAULT_VAL and not self._message_timed_out

  @property
  def model_name(self):
    return self._get_val(self._current_message.model_name)

  @property
  def model_number(self):
    return self._get_val(self._current_message.model_number)

  @property
  def serial_number(self):
    return self._get_val(self._current_message.serial_number)

  @property
  def options(self):
    return self._get_val(self._current_message.options)

  @property
  def firmware_version(self):
    return self._get_val(self._current_message.firmware_version)

  @property
  def model_name_string(self):
    return self._get_string(self.model_name)

  @property
  def model_number_string(self):
    return self._get_string(self.model_number)

  @property
  def serial_number_string(self):
    return self._get_string(self.serial_number)

  @property
  def options_string(self):
    return self._get_string(self.options)

  @property
  def firmware_version_string(self):
    return self._get_string(self.firmware_version)

  @property
  def connected_string(self):
    return self._get_small_boolean_icon_string(self.connected)
