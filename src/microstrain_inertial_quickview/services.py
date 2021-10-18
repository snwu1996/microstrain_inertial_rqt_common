import os
import rospy
from threading import Thread

from microstrain_inertial_msgs.srv import DeviceReport

from .common import ServiceMonitor
from .constants import _DEFAULT_VAL

class DeviceReportMonitor(ServiceMonitor):

  def __init__(self, node_name: str) -> None:
    super().__init__(node_name, "device_report", DeviceReport, message_timeout=2, callback=self._callback)

    # Set the connected state to false by default
    self.connected = False

  def _callback(self, event):
    # Get the result of the service call
    new_message = self._microstrain_services.call_service(self._monitor_path, self._service_type)

    # If the service returned an actual value, save the time of success
    if new_message is not _DEFAULT_VAL:
      self._current_message = new_message
      self._last_message_received_time = rospy.Time.now().to_sec()

      # If the service call succeeded, set the connected flag
      self.connected = self._current_message is not _DEFAULT_VAL

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
