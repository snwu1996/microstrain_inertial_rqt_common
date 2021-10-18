import os
import rospy
from rospy import service

from .constants import _DEFAULT_MESSAGE_TIMEOUT, _DEFAULT_POLL_INTERVAL, _DEFAULT_VAL, _DEFAULT_STR
from .constants import _ICON_GREY_UNCHECKED_SMALL, _ICON_GREEN_UNCHECKED_SMALL

class MicrostrainServices:
  
  def __init__(self, node_name):
    # Save the node name to the class so we can use it later
    self._node_name = node_name

  def call_service(self, service_path, service_type, *args):
    # Wait for the service to become available
    service_name = os.path.join(self._node_name, service_path)
    try:
      rospy.wait_for_service(service_name, timeout=5)
    except (rospy.ROSException, rospy.service.ServiceException, rospy.exceptions.TransportException):
      return _DEFAULT_VAL

    # Call the service, and return the result
    service_func = rospy.ServiceProxy(service_name, service_type)
    response = service_func(*args)
    if hasattr(response, 'success'):
      if response.success:
        return response
      else:
        return _DEFAULT_VAL
    else:
      return response


class Monitor:

  def __init__(self, node_name: str, path: str, message_type, message_timeout = _DEFAULT_MESSAGE_TIMEOUT) -> None:
    # Set up some common variables
    self._node_name = node_name
    self._monitor_path = path
    self._last_message_received_time = 0
    self._message_timeout = message_timeout
    self._current_message = message_type()

  def stop(self):
    self._last_message_received_time = 0

  @property
  def _message_timed_out(self):
    time_elapsed_since_last_message = rospy.Time.now().to_sec() - self._last_message_received_time
    return time_elapsed_since_last_message > self._message_timeout
  
  def _get_val(self, val, default = _DEFAULT_VAL):
    if self._message_timed_out:
      return default
    else:
      return val
  
  def _get_string(self, val, default_val = _DEFAULT_VAL, default_str = _DEFAULT_STR):
    if val is default_val or self._get_val(val, default_val) is default_val:
      return default_str
    else:
      return str(val)

  def _get_string_units(self, val, units, default_val=_DEFAULT_VAL, default_str=_DEFAULT_STR):
    if val is default_val or self._get_val(val, default_val) is default_val:
      return default_str
    else:
      return "%.6f %s" % (val, units)
  
  def _get_small_boolean_icon_string(self, val, default_val = _DEFAULT_VAL):
    if val is default_val or self._get_val(val, default_val) is default_val:
      return _ICON_GREY_UNCHECKED_SMALL
    else:
      if val:
        return _ICON_GREEN_UNCHECKED_SMALL
      else:
        return _ICON_GREY_UNCHECKED_SMALL


class ServiceMonitor(Monitor):
  
  def __init__(self, node_name: str, path: str, service_type, message_timeout=_DEFAULT_MESSAGE_TIMEOUT, callback=_DEFAULT_VAL, poll_interval=_DEFAULT_POLL_INTERVAL) -> None:
    # Initialize the parent class
    super().__init__(node_name, path, service_type._response_class, message_timeout=message_timeout)

    # Save some important information about the services
    self._service_type = service_type

    # Initialize a client that we will use to communicate with the services
    self._microstrain_services = MicrostrainServices(node_name)

    # Start polling the service in a timer
    if callback is _DEFAULT_VAL:
      callback = self._default_callback
    self._poll_timer = rospy.Timer(rospy.Duration(poll_interval), callback)
  
  def stop(self):
    super().stop()
    self._poll_timer.shutdown()
    del self._poll_timer

  def _default_callback(self, event):
    # Get the result of the service call
    new_message = self._microstrain_services.call_service(self._monitor_path, self._service_type)

    # If the service returned an actual value, save the time of success
    if new_message is not _DEFAULT_VAL:
      self._current_message = new_message
      self._last_message_received_time = rospy.Time.now().to_sec()


class SubscriberMonitor(Monitor):

  def __init__(self, node_name: str, path: str, message_type, message_timeout=_DEFAULT_MESSAGE_TIMEOUT, callback=_DEFAULT_VAL) -> None:
    # Initialize the parent class
    super().__init__(node_name, path, message_type, message_timeout=message_timeout)

    # Save th topic name
    self._topic = os.path.join(self._node_name, self._monitor_path)

    # Set up the subscriber
    if callback is _DEFAULT_VAL:
      callback = self._default_callback
    self._subscriber = rospy.Subscriber(self._topic, message_type, callback)

  def stop(self):
    super().stop()
    self._subscriber.unregister()
    del self._subscriber

  def _default_callback(self, message):
    self._current_message = message
    self._last_message_received_time = rospy.Time.now().to_sec()