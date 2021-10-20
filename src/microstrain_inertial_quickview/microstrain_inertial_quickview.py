import os
import re
import rospy
import rosgraph
import python_qt_binding
from qt_gui.plugin import Plugin
from .constants import _DEFAULT_STR, _RESOURCE_DIR_NAME, _PACKAGE_DIR
from .services import DeviceReportMonitor
from .subscribers import GNSSAidingStatusMonitor, GNSSDualAntennaStatusMonitor, GNSSFixInfoMonitor, FilterStatusMonitor, RTKMonitor, OdomMonitor, ImuMonitor, MagMonitor, FilterAidingMeasurementSummaryMonitor, GQ7LedMonitor

_NODE_NAME_ENV_KEY = "MICROSTRAIN_INERTIAL_QUICKVIEW_NODE_NAME"
_DEFAULT_NODE_NAME = "gx5"


class MicrostrainInertialQuickview(Plugin):

  def __init__(self, context):
    # Initialize the parent class
    super(MicrostrainInertialQuickview, self).__init__(context)

    # Give QObjects reasonable names
    self.setObjectName('MicrostainInertialQuickview')

    # Load the UI spec file into a QWidget
    self._widget = python_qt_binding.QtWidgets.QWidget()
    ui_file = os.path.join(_PACKAGE_DIR, _RESOURCE_DIR_NAME, 'MicrostrainInertialQuickview.ui')
    python_qt_binding.loadUi(ui_file, self._widget)
    self._widget.setObjectName('MicrostrainInertialQuickview')

    # Change the name of the widget
    self._widget.setWindowTitle('MicrostrainInertialQuickview')

    # Show _widget.windowTitle on left-top of each plugin (when 
    # it's set in _widget). This is useful when you open multiple 
    # plugins at once. Also if you open multiple instances of your 
    # plugin at once, these lines add number to make it easy to 
    # tell from pane to pane.
    if context.serial_number() > 1:
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

    # Add widget to the user interface
    context.add_widget(self._widget)

    # Since this is loaded as an RQT plugin, we can't get config from a launch file. Instead, read an environment variable to determine what the default node name should be
    self._node_name = os.getenv(_NODE_NAME_ENV_KEY, _DEFAULT_NODE_NAME)
    if self._widget.node_name_line_edit.text() == _DEFAULT_STR:  # If loading the default perspective, set the node name in the UI
      self._widget.node_name_line_edit.setText(self._node_name)
    else:
      self._node_name = self._widget.node_name_line_edit.text()

    # Register the button callbacks
    self._widget.node_name_submit_button.clicked.connect(self.set_node_name)

    # Set up the monitors with the default node name
    self._setup_monitors(self._node_name)

    # Start the UI update loop
    self._ui_update_timer = python_qt_binding.QtCore.QTimer()
    self._ui_update_timer.setSingleShot(False)
    self._ui_update_timer.timeout.connect(self.update_display_data)
    self._ui_update_timer.start(250)

  def shutdown_plugin(self):
    # Stop the update loop
    self._ui_update_timer.stop()
    del self._ui_update_timer

    # Stop the monitors
    self._device_report_monitor.stop()
    self._imu_monitor.stop()
    self._mag_monitor.stop()
    self._absolute_odom_monitor.stop()
    self._relative_odom_monitor.stop()
    self._filter_aiding_status_summary_monitor.stop()
    self._gnss_1_aiding_status_monitor.stop()
    self._gnss_2_aiding_status_monitor.stop()
    self._gnss_1_fix_info_monitor.stop()
    self._gnss_2_fix_info_monitor.stop()
    self._gnss_dual_antenna_status_monitor.stop()
    self._filter_status_monitor.stop()
    self._rtk_status_monitor.stop()

    # Delete all the monitors
    del self._device_report_monitor
    del self._imu_monitor
    del self._mag_monitor
    del self._absolute_odom_monitor
    del self._relative_odom_monitor
    del self._filter_aiding_status_summary_monitor
    del self._gnss_1_aiding_status_monitor
    del self._gnss_2_aiding_status_monitor
    del self._gnss_1_fix_info_monitor
    del self._gnss_2_fix_info_monitor
    del self._gnss_dual_antenna_status_monitor
    del self._filter_status_monitor
    del self._rtk_status_monitor
    del self._gq7_led_monitor

    # Call the parent function
    return super(MicrostrainInertialQuickview, self).shutdown_plugin()

  def set_node_name(self):
    # Get the value from the node name text and reinitialize the monitors
    self._node_name = self._widget.node_name_line_edit.text()
    self._setup_monitors(self._node_name)

  def _setup_monitors(self, node_name):
    # Sanitize the node name
    node_name = re.sub(r'\/+', '/', node_name)  # Remove duplicate slashes
    if not node_name.startswith('/'):  # Start with a slash
      node_name = '/' + node_name
    if node_name.endswith('/'):  # Do not end with a slash
      node_name = node_name[:-1]

    # Set up the service status monitors
    self._device_report_monitor = DeviceReportMonitor(node_name)

    # Set up the subscriber status monitors
    self._imu_monitor = ImuMonitor(node_name, "imu/data")
    self._mag_monitor = MagMonitor(node_name, "mag")
    self._absolute_odom_monitor = OdomMonitor(node_name, "nav/odom", llh=True)
    self._relative_odom_monitor = OdomMonitor(node_name, "nav/relative_pos/odom", llh=False)
    self._filter_aiding_status_summary_monitor = FilterAidingMeasurementSummaryMonitor(node_name, "nav/aiding_summary")
    self._gnss_1_aiding_status_monitor = GNSSAidingStatusMonitor(node_name, "gnss1/aiding_status")
    self._gnss_2_aiding_status_monitor = GNSSAidingStatusMonitor(node_name, "gnss2/aiding_status")
    self._gnss_1_fix_info_monitor = GNSSFixInfoMonitor(node_name, "gnss1/fix_info")
    self._gnss_2_fix_info_monitor = GNSSFixInfoMonitor(node_name, "gnss2/fix_info")
    self._gnss_dual_antenna_status_monitor = GNSSDualAntennaStatusMonitor(node_name, "nav/dual_antenna_status")
    self._filter_status_monitor = FilterStatusMonitor(node_name, "nav/status")
    self._rtk_status_monitor = RTKMonitor(node_name, "rtk/status")

    # Set up a special monitor for the GQ7 LED
    self._gq7_led_monitor = GQ7LedMonitor(self._filter_status_monitor, self._gnss_1_aiding_status_monitor, self._gnss_2_aiding_status_monitor)

  def update_display_data(self):
    self._update_device_report()
    self._update_gq7_led()
    self._update_imu()
    self._update_mag()
    self._update_odom()
    self._update_aiding_measurements_data()
    self._update_gnss_1_data()
    self._update_gnss_2_data()
    self._update_dual_antenna_data()
    self._update_filter_status_data()
    self._update_rtk_data()

  def _update_device_report(self):
    self._widget.node_connected_label.setText(self._device_report_monitor.connected_string)
    self._widget.model_name_label.setText(self._device_report_monitor.model_name_string)
    self._widget.model_number_label.setText(self._device_report_monitor.model_number_string)
    self._widget.serial_number_label.setText(self._device_report_monitor.serial_number_string)
    self._widget.options_label.setText(self._device_report_monitor.options_string)
    self._widget.firmware_version_label.setText(self._device_report_monitor.firmware_version_string)

  def _update_gq7_led(self):
    self._widget.gq7_led_status_label.setText(self._filter_status_monitor.filter_state_led_string)
    self._widget.gq7_led_icon_label.setText(self._gq7_led_monitor.gq7_led_icon)

  def _update_imu(self):
    # Accel
    self._widget.sensor_accel_x_label.setText(self._imu_monitor.accel_x_string)
    self._widget.sensor_accel_y_label.setText(self._imu_monitor.accel_y_string)
    self._widget.sensor_accel_z_label.setText(self._imu_monitor.accel_z_string)

    # Gyro
    self._widget.sensor_gyro_x_label.setText(self._imu_monitor.vel_x_string)
    self._widget.sensor_gyro_y_label.setText(self._imu_monitor.vel_y_string)
    self._widget.sensor_gyro_z_label.setText(self._imu_monitor.vel_z_string)

  def _update_mag(self):
    self._widget.sensor_magnetometer_x_label.setText(self._mag_monitor.x_string)
    self._widget.sensor_magnetometer_y_label.setText(self._mag_monitor.y_string)
    self._widget.sensor_magnetometer_z_label.setText(self._mag_monitor.z_string)

  def _update_odom(self):
    # Absolute Position
    self._widget.filter_position_lat_label.setText(self._absolute_odom_monitor.position_x_string)
    self._widget.filter_position_lon_label.setText(self._absolute_odom_monitor.position_y_string)
    self._widget.filter_position_alt_label.setText(self._absolute_odom_monitor.position_z_string)

    # Absolute Position Uncertainty
    self._widget.filter_position_lat_uncertainty_label.setText(self._absolute_odom_monitor.position_uncertainty_x_string)
    self._widget.filter_position_lon_uncertainty_label.setText(self._absolute_odom_monitor.position_uncertainty_y_string)
    self._widget.filter_position_alt_uncertainty_label.setText(self._absolute_odom_monitor.position_uncertainty_z_string)

    # Orientation
    self._widget.filter_roll_label.setText(self._absolute_odom_monitor.orientation_x_string)
    self._widget.filter_pitch_label.setText(self._absolute_odom_monitor.orientation_y_string)
    self._widget.filter_yaw_label.setText(self._absolute_odom_monitor.orientation_z_string)

    # Orientation Uncertainty
    self._widget.filter_roll_uncertainty_label.setText(self._absolute_odom_monitor.orientation_uncertainty_x_string)
    self._widget.filter_pitch_uncertainty_label.setText(self._absolute_odom_monitor.orientation_uncertainty_y_string)
    self._widget.filter_yaw_uncertainty_label.setText(self._absolute_odom_monitor.orientation_uncertainty_z_string)

    # Relative Position
    self._widget.filter_relative_position_lat_label.setText(self._relative_odom_monitor.position_x_string)
    self._widget.filter_relative_position_lon_label.setText(self._relative_odom_monitor.position_y_string)
    self._widget.filter_relative_position_alt_label.setText(self._relative_odom_monitor.position_z_string)

    # Relative Position Uncertainty
    self._widget.filter_relative_position_lat_uncertainty_label.setText(self._relative_odom_monitor.position_uncertainty_x_string)
    self._widget.filter_relative_position_lon_uncertainty_label.setText(self._relative_odom_monitor.position_uncertainty_y_string)
    self._widget.filter_relative_position_alt_uncertainty_label.setText(self._relative_odom_monitor.position_uncertainty_z_string)

  def _update_aiding_measurements_data(self):
    self._widget.filter_aiding_measurements_gnss_1_enabled.setText(self._filter_aiding_status_summary_monitor.gnss1_enabled_string)
    self._widget.filter_aiding_measurements_gnss_1_used.setText(self._filter_aiding_status_summary_monitor.gnss1_used_string)
    self._widget.filter_aiding_measurements_gnss_2_enabled.setText(self._filter_aiding_status_summary_monitor.gnss2_enabled_string)
    self._widget.filter_aiding_measurements_gnss_2_used.setText(self._filter_aiding_status_summary_monitor.gnss2_used_string)
    self._widget.filter_aiding_measurements_dual_antenna_enabled.setText(self._filter_aiding_status_summary_monitor.dual_antenna_enabled_string)
    self._widget.filter_aiding_measurements_dual_antenna_used.setText(self._filter_aiding_status_summary_monitor.dual_antenna_used_string)
    self._widget.filter_aiding_measurements_heading_enabled.setText(self._filter_aiding_status_summary_monitor.heading_enabled_string)
    self._widget.filter_aiding_measurements_heading_used.setText(self._filter_aiding_status_summary_monitor.heading_used_string)
    self._widget.filter_aiding_measurements_pressure_enabled.setText(self._filter_aiding_status_summary_monitor.pressure_enabled_string)
    self._widget.filter_aiding_measurements_pressure_used.setText(self._filter_aiding_status_summary_monitor.pressure_used_string)
    self._widget.filter_aiding_measurements_mag_enabled.setText(self._filter_aiding_status_summary_monitor.magnetometer_enabled_string)
    self._widget.filter_aiding_measurements_mag_used.setText(self._filter_aiding_status_summary_monitor.magnetometer_used_string)
    self._widget.filter_aiding_measurements_speed_enabled.setText(self._filter_aiding_status_summary_monitor.speed_enabled_string)
    self._widget.filter_aiding_measurements_speed_used.setText(self._filter_aiding_status_summary_monitor.speed_used_string)

  def _update_gnss_1_data(self):
    # Flags
    self._widget.filter_position_aiding_gnss_1_tight_coupling_label.setText(self._gnss_1_aiding_status_monitor.tight_coupling_string)
    self._widget.filter_position_aiding_gnss_1_differential_label.setText(self._gnss_1_aiding_status_monitor.differential_corrections_string)
    self._widget.filter_position_aiding_gnss_1_integer_fix_label.setText(self._gnss_1_aiding_status_monitor.integer_fix_string)

    # Fix Info
    self._widget.gnss_1_fix_type_label.setText(self._gnss_1_fix_info_monitor.fix_type_string)
    self._widget.gnss_1_sv_count_label.setText(self._gnss_1_fix_info_monitor.num_sv_string)

    # TODO(robbiefish): Add Position Uncertainty here

  def _update_gnss_2_data(self):
    # Flags
    self._widget.filter_position_aiding_gnss_2_tight_coupling_label.setText(self._gnss_2_aiding_status_monitor.tight_coupling_string)
    self._widget.filter_position_aiding_gnss_2_differential_label.setText(self._gnss_2_aiding_status_monitor.differential_corrections_string)
    self._widget.filter_position_aiding_gnss_2_integer_fix_label.setText(self._gnss_2_aiding_status_monitor.integer_fix_string)

    # Fix Info
    self._widget.gnss_2_fix_type_label.setText(self._gnss_2_fix_info_monitor.fix_type_string)
    self._widget.gnss_2_sv_count_label.setText(self._gnss_2_fix_info_monitor.num_sv_string)

  def _update_dual_antenna_data(self):
    self._widget.filter_dual_antenna_fix_type_label.setText(self._gnss_dual_antenna_status_monitor.fix_type_string)
    self._widget.filter_dual_antenna_heading_label.setText(self._gnss_dual_antenna_status_monitor.heading_string)
    self._widget.filter_dual_antenna_uncertainty_label.setText(self._gnss_dual_antenna_status_monitor.heading_uncertainty_string)
    self._widget.filter_dual_antenna_rec_1_data_valid_label.setText(self._gnss_dual_antenna_status_monitor.rec_1_data_valid_string)
    self._widget.filter_dual_antenna_rec_2_data_valid_label.setText(self._gnss_dual_antenna_status_monitor.rec_2_data_valid_string)
    self._widget.filter_dual_antenna_offsets_valid_label.setText(self._gnss_dual_antenna_status_monitor.antenna_offsets_valid_string)

  def _update_filter_status_data(self):
    self._widget.filter_state_label.setText(self._filter_status_monitor.filter_state_string)
    self._widget.filter_status_flags_label.setText(self._filter_status_monitor.status_flags_string)
  
  def _update_rtk_data(self):
    # Epoch Status flags
    self._widget.rtk_corrections_received_gps_label.setText(self._rtk_status_monitor.gps_received_string)
    self._widget.rtk_corrections_received_glonass_label.setText(self._rtk_status_monitor.glonass_received_string)
    self._widget.rtk_corrections_received_galileo_label.setText(self._rtk_status_monitor.galileo_received_string)
    self._widget.rtk_corrections_received_beidou_label.setText(self._rtk_status_monitor.beidou_received_string)

    # Icon and label
    self._widget.rtk_led_icon_label.setText(self._rtk_status_monitor.rtk_led_string)
    self._widget.rtk_led_status_label.setText(self._rtk_status_monitor.controller_status_string)

    # Other Status Flags
    self._widget.rtk_status_flags_mode_label.setText(self._rtk_status_monitor.controller_state_string)
    self._widget.rtk_status_flags_controller_status_label.setText(self._rtk_status_monitor.controller_status_string)
    self._widget.rtk_status_flags_device_state_label.setText(self._rtk_status_monitor.platform_state_string)
    self._widget.rtk_status_flags_connection_status_label.setText(self._rtk_status_monitor.platform_status_string)
    self._widget.rtk_status_flags_reset_reason_label.setText(self._rtk_status_monitor.reset_reason_string)
    self._widget.rtk_status_flags_signal_quality_label.setText(self._rtk_status_monitor.signal_quality_string)