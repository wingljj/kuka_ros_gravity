#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sriforcesensor/SetFilterConfig.h>
#include <tool_gravity_compensation/ComputePayload.h>
#include <tool_gravity_compensation/SetSamplingConfig.h>
#include <tool_gravity_compensation/SetSensorMount.h>
#include <tool_gravity_compensation/StepControl.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace tool_gravity_compensation
{
namespace
{
QDoubleSpinBox* makeDoubleSpin(double min, double max, double value, int decimals = 4)
{
  auto* spin = new QDoubleSpinBox;
  spin->setRange(min, max);
  spin->setDecimals(decimals);
  spin->setSingleStep(decimals >= 4 ? 0.001 : 0.01);
  spin->setValue(value);
  return spin;
}

QSpinBox* makeIntSpin(int min, int max, int value)
{
  auto* spin = new QSpinBox;
  spin->setRange(min, max);
  spin->setValue(value);
  return spin;
}
}  // namespace

class ToolGravityPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit ToolGravityPanel(QWidget* parent = nullptr)
    : rviz::Panel(parent)
    , nh_()
  {
    auto* main_layout = new QVBoxLayout;
    main_layout->setSpacing(2);
    main_layout->setContentsMargins(2, 2, 2, 2);

    // ── Header: logo + title (single line) ──
    main_layout->addWidget(createHeader());

    // ── Tool position bar (compact horizontal) ──
    main_layout->addWidget(createToolPositionBar());

    // ── Two-column settings area ──
    auto* columns = new QHBoxLayout;
    columns->setSpacing(4);
    auto* left_col = new QVBoxLayout;
    auto* right_col = new QVBoxLayout;
    left_col->setSpacing(2);
    right_col->setSpacing(2);

    left_col->addWidget(createConnectionGroup());
    left_col->addWidget(createMountGroup());
    left_col->addStretch();

    right_col->addWidget(createSamplingGroup());
    right_col->addWidget(createSafetyGroup());
    right_col->addStretch();

    columns->addLayout(left_col, 1);
    columns->addLayout(right_col, 1);
    main_layout->addLayout(columns);

    // ── Full-width acquisition ──
    main_layout->addWidget(createAcquisitionGroup());

    // ── Compact status line ──
    status_label_ = new QLabel("就绪。需要手动确认。");
    status_label_->setWordWrap(true);
    status_label_->setStyleSheet("QLabel { color: #333; font-weight: bold; padding: 1px 2px; font-size: 11px; }");
    main_layout->addWidget(status_label_);

    // ── Operation log (compact) ──
    main_layout->addWidget(createLogGroup());

    setLayout(main_layout);

    // ── Service clients ──
    step_client_ = nh_.serviceClient<StepControl>("step_control");
    compute_client_ = nh_.serviceClient<ComputePayload>("compute_payload");
    sensor_mount_client_ = nh_.serviceClient<SetSensorMount>("set_sensor_mount");
    sampling_config_client_ = nh_.serviceClient<SetSamplingConfig>("set_sampling_config");
    sri_filter_client_ = nh_.serviceClient<sriforcesensor::SetFilterConfig>("sri_ft_sensor/set_filter_config");

    // ── Wrench monitor subscriber (for data-rate detection) ──
    wrench_monitor_sub_ = nh_.subscribe("/sri_ft_sensor/wrench", 10,
                                        &ToolGravityPanel::handleWrenchMonitor, this);

    // ── Signal/slot connections ──
    connect(apply_mount_button_, SIGNAL(clicked()), this, SLOT(onApplyMountClicked()));
    connect(check_tf_button_, SIGNAL(clicked()), this, SLOT(onCheckTfClicked()));
    connect(apply_sampling_button_, SIGNAL(clicked()), this, SLOT(onApplySamplingClicked()));
    connect(tool_button_, SIGNAL(clicked()), this, SLOT(onToolOnlyClicked()));
    connect(total_button_, SIGNAL(clicked()), this, SLOT(onToolPlusPayloadClicked()));
    connect(compute_button_, SIGNAL(clicked()), this, SLOT(onComputeClicked()));
    connect(clear_profiles_button_, SIGNAL(clicked()), this, SLOT(onClearProfilesClicked()));
    connect(sensor_connect_button_, SIGNAL(clicked()), this, SLOT(onConnectSensorClicked()));
    connect(robot_connect_button_, SIGNAL(clicked()), this, SLOT(onConnectRobotClicked()));
    connect(clear_log_button_, SIGNAL(clicked()), this, SLOT(onClearLogClicked()));

    connect(estop_check_, SIGNAL(stateChanged(int)), this, SLOT(updateSafetyGate()));
    connect(teach_pendant_check_, SIGNAL(stateChanged(int)), this, SLOT(updateSafetyGate()));
    connect(fence_check_, SIGNAL(stateChanged(int)), this, SLOT(updateSafetyGate()));
    connect(low_speed_check_, SIGNAL(stateChanged(int)), this, SLOT(updateSafetyGate()));
    connect(tool_fixed_check_, SIGNAL(stateChanged(int)), this, SLOT(updateSafetyGate()));

    updateSafetyGate();
    refresh_timer_ = new QTimer(this);
    connect(refresh_timer_, SIGNAL(timeout()), this, SLOT(onRefreshStatusClicked()));
    refresh_timer_->start(1000);

    logOperation("上海卫星装备研究所 · 机械臂负载测试系统 v2.0 已就绪");
  }

private Q_SLOTS:
  void onApplyMountClicked()
  {
    logOperation("▶ 应用传感器安装参数...");
    SetSensorMount srv;
    srv.request.parent_frame = "tool0";
    srv.request.child_frame = "sri_ft_sensor";
    srv.request.x = mount_x_->value();
    srv.request.y = mount_y_->value();
    srv.request.z = mount_z_->value();
    srv.request.roll = mount_roll_->value() * M_PI / 180.0;
    srv.request.pitch = mount_pitch_->value() * M_PI / 180.0;
    srv.request.yaw = mount_yaw_->value() * M_PI / 180.0;
    srv.request.save_to_params = true;

    if (!sensor_mount_client_.waitForExistence(ros::Duration(0.2)) || !sensor_mount_client_.call(srv))
    {
      status_label_->setText("set_sensor_mount 服务不可用。");
      logOperation("✗ set_sensor_mount 服务不可用");
      return;
    }
    QString msg = QString::fromStdString(srv.response.message);
    status_label_->setText(msg);
    logOperation("✓ " + msg);
  }

  void onCheckTfClicked()
  {
    logOperation("▶ 检查 TF tool0 → sri_ft_sensor...");
    tf::StampedTransform transform;
    try
    {
      tf_listener_.lookupTransform("tool0", "sri_ft_sensor", ros::Time(0), transform);
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(4)
             << "TF正常 tool0 -> sri_ft_sensor xyz=["
             << transform.getOrigin().x() << ", "
             << transform.getOrigin().y() << ", "
             << transform.getOrigin().z() << "]";
      status_label_->setText(QString::fromStdString(stream.str()));
      logOperation("✓ " + QString::fromStdString(stream.str()));
    }
    catch (const tf::TransformException& ex)
    {
      QString msg = QString("TF缺失: ") + ex.what();
      status_label_->setText(msg);
      logOperation("✗ " + msg);
    }
  }

  void onApplySamplingClicked()
  {
    logOperation("▶ 应用滤波/采样配置...");
    SetSamplingConfig srv;
    srv.request.apply = true;
    srv.request.clear_profiles = false;
    srv.request.filter_window_size = filter_window_spin_->value();
    srv.request.min_stable_samples = min_stable_samples_spin_->value();
    srv.request.max_stddev_force = max_stddev_force_spin_->value();
    srv.request.max_stddev_torque = max_stddev_torque_spin_->value();
    srv.request.sample_timeout = sample_timeout_spin_->value();

    nh_.setParam("/sri_forcesensor/filter/enabled", filter_enabled_check_->isChecked());
    nh_.setParam("/sri_forcesensor/filter/window_size", filter_window_spin_->value());
    nh_.setParam("/sri_forcesensor/filter/max_stddev_force", max_stddev_force_spin_->value());
    nh_.setParam("/sri_forcesensor/filter/max_stddev_torque", max_stddev_torque_spin_->value());
    nh_.setParam("/sri_forcesensor/sensor_ip", sensor_ip_edit_->text().toStdString());
    nh_.setParam("/sri_forcesensor/publish_rate", publish_rate_spin_->value());
    nh_.setParam("/kuka_eki/robot_ip", robot_ip_edit_->text().toStdString());
    nh_.setParam("/kuka_eki/port", eki_port_spin_->value());
    nh_.setParam("/tool_gravity_compensation/run_mode", mode_combo_->currentText().toStdString());

    bool sri_filter_ok = false;
    sriforcesensor::SetFilterConfig filter_srv;
    filter_srv.request.apply = true;
    filter_srv.request.enabled = filter_enabled_check_->isChecked();
    filter_srv.request.window_size = filter_window_spin_->value();
    filter_srv.request.max_stddev_force = max_stddev_force_spin_->value();
    filter_srv.request.max_stddev_torque = max_stddev_torque_spin_->value();
    if (sri_filter_client_.waitForExistence(ros::Duration(0.2)))
    {
      sri_filter_ok = sri_filter_client_.call(filter_srv) && filter_srv.response.success;
    }

    if (!sampling_config_client_.waitForExistence(ros::Duration(0.2)) || !sampling_config_client_.call(srv))
    {
      status_label_->setText("set_sampling_config 服务不可用。");
      logOperation("✗ set_sampling_config 服务不可用");
      return;
    }

    std::ostringstream status;
    status << srv.response.message;
    if (mode_combo_->currentIndex() == 0)
    {
      status << " | 离线模式下跳过 SRI 滤波服务";
    }
    else if (sri_filter_ok)
    {
      status << " | " << filter_srv.response.message;
    }
    else
    {
      status << " | SRI 滤波服务缺失或被拒绝";
    }
    QString msg = QString::fromStdString(status.str());
    status_label_->setText(msg);
    logOperation("✓ " + msg);
  }

  void onToolOnlyClicked()
  {
    logOperation("▶ 记录 TOOL_ONLY 样本...");
    recordSample(StepControl::Request::TOOL_ONLY, "TOOL_ONLY");
  }

  void onToolPlusPayloadClicked()
  {
    logOperation("▶ 记录 TOOL_PLUS_PAYLOAD 样本...");
    recordSample(StepControl::Request::TOOL_PLUS_PAYLOAD, "TOOL_PLUS_PAYLOAD");
  }

  void onComputeClicked()
  {
    logOperation("▶ 计算负载参数...");
    ComputePayload srv;
    srv.request.compute = true;
    srv.request.min_samples = static_cast<uint32_t>(min_samples_spin_->value());

    if (!compute_client_.waitForExistence(ros::Duration(0.2)) || !compute_client_.call(srv))
    {
      status_label_->setText("compute_payload 服务不可用。");
      logOperation("✗ compute_payload 服务不可用");
      return;
    }

    QString msg = QString::fromStdString(srv.response.message);
    status_label_->setText(msg);
    if (!srv.response.success)
    {
      logOperation("✗ " + msg);
      return;
    }

    std::ostringstream result;
    result << std::fixed << std::setprecision(4)
           << "负载质量: " << srv.response.result.mass_kg << " kg\n"
           << "负载重量: " << srv.response.result.weight_n << " N\n"
           << "负载质心 (sri_ft_sensor系): ["
           << srv.response.result.com_sensor.x << ", "
           << srv.response.result.com_sensor.y << ", "
           << srv.response.result.com_sensor.z << "] m\n"
           << "工具质量: " << srv.response.result.tool_mass << " kg";
    QString result_str = QString::fromStdString(result.str());
    status_label_->setText("✓ 负载计算完成");
    logOperation("✓ 负载计算完成:\n" + result_str);
    ROS_INFO_STREAM("负载结果: " << result.str());
  }

  void onClearProfilesClicked()
  {
    logOperation("▶ 清除采集配置...");
    SetSamplingConfig srv;
    srv.request.apply = true;
    srv.request.clear_profiles = true;
    srv.request.filter_window_size = filter_window_spin_->value();
    srv.request.min_stable_samples = min_stable_samples_spin_->value();
    srv.request.max_stddev_force = max_stddev_force_spin_->value();
    srv.request.max_stddev_torque = max_stddev_torque_spin_->value();
    srv.request.sample_timeout = sample_timeout_spin_->value();

    if (!sampling_config_client_.waitForExistence(ros::Duration(0.2)) || !sampling_config_client_.call(srv))
    {
      status_label_->setText("set_sampling_config 服务不可用。");
      logOperation("✗ set_sampling_config 服务不可用");
      return;
    }
    next_step_index_ = 0;
    QString msg = QString::fromStdString(srv.response.message);
    status_label_->setText(msg);
    logOperation("✓ 配置已清除。请先记录仅工具，再记录工具加负载。");
  }

  void onClearLogClicked()
  {
    log_view_->clear();
    logOperation("日志已清除");
  }

  void updateSafetyGate()
  {
    const bool safe = estop_check_->isChecked() &&
                      teach_pendant_check_->isChecked() &&
                      fence_check_->isChecked() &&
                      low_speed_check_->isChecked() &&
                      tool_fixed_check_->isChecked();
    const bool was_safe = tool_button_->isEnabled();
    tool_button_->setEnabled(safe);
    total_button_->setEnabled(safe);
    compute_button_->setEnabled(safe);
    safety_label_->setText(safe ? "安全检查已完成。" :
                                  "采样前请完成安全检查。");
    if (safe != was_safe)
    {
      logOperation(safe ? "✓ 安全检查已全部通过，采样已启用" :
                          "⚠ 安全检查未完成，采样已禁用");
    }
  }

  void onRefreshStatusClicked()
  {
    // Update tool position readout
    updateToolPosition();

    // ── Sensor status: check if wrench messages are arriving ──
    const double dt = (ros::Time::now() - last_wrench_stamp_).toSec();
    const bool sensor_data_flowing = (dt < 2.0 && last_wrench_stamp_ != ros::Time(0));
    if (sensor_data_flowing)
    {
      double hz = (dt > 0.001) ? (1.0 / dt) : 0.0;
      sensor_status_label_->setText(QString("● 已连接 (%1 Hz)").arg(hz, 0, 'f', 0));
      sensor_status_label_->setStyleSheet("QLabel { color: #4CAF50; font-size: 10px; font-weight: bold; }");
      sensor_connect_button_->setChecked(true);
      sensor_connect_button_->setText("已连接");
    }
    else if (sensor_connect_button_->isChecked())
    {
      sensor_status_label_->setText("● 检测中...");
      sensor_status_label_->setStyleSheet("QLabel { color: #FF9800; font-size: 10px; }");
    }

    // ── Robot status: check if step_control service exists ──
    const bool robot_ok = step_client_.exists();
    if (robot_ok)
    {
      robot_status_label_->setText("● 已连接");
      robot_status_label_->setStyleSheet("QLabel { color: #4CAF50; font-size: 10px; font-weight: bold; }");
      robot_connect_button_->setChecked(true);
      robot_connect_button_->setText("已连接");
    }
    else if (robot_connect_button_->isChecked())
    {
      robot_status_label_->setText("● 检测中...");
      robot_status_label_->setStyleSheet("QLabel { color: #FF9800; font-size: 10px; }");
    }
  }

  void handleWrenchMonitor(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    last_wrench_stamp_ = msg->header.stamp;
  }

  void onConnectSensorClicked()
  {
    if (sensor_connect_button_->isChecked())
    {
      logOperation("▶ 尝试连接 SRI 力传感器 " + sensor_ip_edit_->text() + "...");
      sensor_status_label_->setText("● 检测中...");
      sensor_status_label_->setStyleSheet("QLabel { color: #FF9800; font-size: 10px; }");
      sensor_connect_button_->setText("检测中...");
      // Store sensor IP to param server
      nh_.setParam("/sri_forcesensor/sensor_ip", sensor_ip_edit_->text().toStdString());
      nh_.setParam("/sri_forcesensor/publish_rate", publish_rate_spin_->value());
      // Connection is verified by the timer checking wrench data flow
      logOperation("  等待传感器数据... (请确保传感器节点已启动)");
    }
    else
    {
      logOperation("▶ 断开传感器连接");
      sensor_status_label_->setText("● 未连接");
      sensor_status_label_->setStyleSheet("QLabel { color: #999; font-size: 10px; }");
      sensor_connect_button_->setText("连接传感器");
      last_wrench_stamp_ = ros::Time(0);
    }
  }

  void onConnectRobotClicked()
  {
    if (robot_connect_button_->isChecked())
    {
      logOperation("▶ 尝试连接 KR240 机器人 " + robot_ip_edit_->text() + ":" +
                   QString::number(eki_port_spin_->value()) + "...");
      robot_status_label_->setText("● 检测中...");
      robot_status_label_->setStyleSheet("QLabel { color: #FF9800; font-size: 10px; }");
      robot_connect_button_->setText("检测中...");
      nh_.setParam("/kuka_eki/robot_ip", robot_ip_edit_->text().toStdString());
      nh_.setParam("/kuka_eki/port", eki_port_spin_->value());
      // Connection is verified by the timer checking step_control service
      logOperation("  等待机器人连接... (请确保 EKI 接口和 MoveIt 已启动)");
    }
    else
    {
      logOperation("▶ 断开机器人连接");
      robot_status_label_->setText("● 未连接");
      robot_status_label_->setStyleSheet("QLabel { color: #999; font-size: 10px; }");
      robot_connect_button_->setText("连接机器人");
    }
  }

private:
  // ── Helper: append timestamped message to operation log ──
  void logOperation(const QString& message)
  {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_view_->appendPlainText("[" + timestamp + "] " + message);
    // Auto-scroll to bottom
    QScrollBar* bar = log_view_->verticalScrollBar();
    bar->setValue(bar->maximum());
  }

  // ═══════════════════════════════════════════════════════════════
  //  Group creators
  // ═══════════════════════════════════════════════════════════════

  QWidget* createHeader()
  {
    auto* header = new QWidget;
    header->setStyleSheet(
      "QWidget#header {"
      "  background: qlineargradient(x1:0, y1:0, x2:1, y2:0,"
      "    stop:0 #0a1f44, stop:0.5 #1a3a6e, stop:1 #0a1f44);"
      "  border-radius: 4px;"
      "}"
    );
    header->setObjectName("header");
    auto* h_layout = new QHBoxLayout;
    h_layout->setContentsMargins(4, 2, 4, 2);
    h_layout->setSpacing(6);

    // ── Compact logo ──
    auto* logo = new QLabel("🚀 中国航天");
    logo->setStyleSheet(
      "QLabel { font-size: 12px; font-weight: bold; color: #d4e4ff;"
      "  background: #0d2b5e; border: 1px solid #3a7bd5;"
      "  border-radius: 3px; padding: 2px 5px; }"
    );

    // ── Title (single line) ──
    auto* title = new QLabel("上海卫星装备研究所 · 机械臂负载测试系统");
    title->setStyleSheet(
      "QLabel { font-size: 13px; font-weight: bold; color: #e8efff;"
      "  border: none; background: transparent; }"
    );

    h_layout->addWidget(logo);
    h_layout->addWidget(title, 1);
    header->setLayout(h_layout);
    return header;
  }

  QGroupBox* createConnectionGroup()
  {
    auto* group = new QGroupBox("设备连接");
    auto* layout = new QVBoxLayout;
    layout->setSpacing(2);

    // ── Mode selector ──
    auto* mode_row = new QHBoxLayout;
    mode_row->addWidget(new QLabel("模式"));
    mode_combo_ = new QComboBox;
    mode_combo_->addItem("离线仿真");
    mode_combo_->addItem("真实传感器");
    mode_combo_->addItem("真实机器人");
    mode_row->addWidget(mode_combo_, 1);
    layout->addLayout(mode_row);

    // ── Sensor connection ──
    auto* sensor_row = new QHBoxLayout;
    sensor_connect_button_ = new QPushButton("连接传感器");
    sensor_connect_button_->setCheckable(true);
    sensor_connect_button_->setStyleSheet(
      "QPushButton { font-weight: bold; padding: 4px 8px; }"
      "QPushButton:checked { background: #4CAF50; color: white; }"
    );
    sensor_status_label_ = new QLabel("● 未连接");
    sensor_status_label_->setStyleSheet("QLabel { color: #999; font-size: 10px; }");
    sensor_ip_edit_ = new QLineEdit("192.168.0.108");
    sensor_ip_edit_->setMaximumWidth(120);
    sensor_row->addWidget(sensor_connect_button_);
    sensor_row->addWidget(new QLabel("SRI IP"));
    sensor_row->addWidget(sensor_ip_edit_);
    sensor_row->addWidget(sensor_status_label_, 1);
    layout->addLayout(sensor_row);

    // ── Robot connection ──
    auto* robot_row = new QHBoxLayout;
    robot_connect_button_ = new QPushButton("连接机器人");
    robot_connect_button_->setCheckable(true);
    robot_connect_button_->setStyleSheet(
      "QPushButton { font-weight: bold; padding: 4px 8px; }"
      "QPushButton:checked { background: #4CAF50; color: white; }"
    );
    robot_status_label_ = new QLabel("● 未连接");
    robot_status_label_->setStyleSheet("QLabel { color: #999; font-size: 10px; }");
    robot_ip_edit_ = new QLineEdit("172.31.1.147");
    robot_ip_edit_->setMaximumWidth(120);
    eki_port_spin_ = makeIntSpin(1, 65535, 54600);
    eki_port_spin_->setMaximumWidth(70);
    publish_rate_spin_ = makeIntSpin(1, 1000, 200);
    publish_rate_spin_->setMaximumWidth(70);
    robot_row->addWidget(robot_connect_button_);
    robot_row->addWidget(new QLabel("IP"));
    robot_row->addWidget(robot_ip_edit_);
    robot_row->addWidget(new QLabel("端口"));
    robot_row->addWidget(eki_port_spin_);
    robot_row->addWidget(new QLabel("Hz"));
    robot_row->addWidget(publish_rate_spin_);
    robot_row->addWidget(robot_status_label_, 1);
    layout->addLayout(robot_row);

    group->setLayout(layout);
    return group;
  }

  QGroupBox* createMountGroup()
  {
    auto* group = new QGroupBox("传感器安装 tool0 → sri_ft_sensor");
    auto* layout = new QGridLayout;
    layout->setHorizontalSpacing(4);
    layout->setVerticalSpacing(2);
    mount_x_ = makeDoubleSpin(-5.0, 5.0, 0.0);
    mount_y_ = makeDoubleSpin(-5.0, 5.0, 0.0);
    mount_z_ = makeDoubleSpin(-5.0, 5.0, 0.0);
    mount_roll_ = makeDoubleSpin(-180.0, 180.0, 0.0, 1);
    mount_pitch_ = makeDoubleSpin(-180.0, 180.0, 0.0, 1);
    mount_yaw_ = makeDoubleSpin(-180.0, 180.0, 0.0, 1);
    apply_mount_button_ = new QPushButton("应用安装");
    check_tf_button_ = new QPushButton("检查TF");
    layout->addWidget(new QLabel("x 米"), 0, 0);
    layout->addWidget(mount_x_, 0, 1);
    layout->addWidget(new QLabel("y 米"), 0, 2);
    layout->addWidget(mount_y_, 0, 3);
    layout->addWidget(new QLabel("z 米"), 0, 4);
    layout->addWidget(mount_z_, 0, 5);
    layout->addWidget(new QLabel("滚转 °"), 1, 0);
    layout->addWidget(mount_roll_, 1, 1);
    layout->addWidget(new QLabel("俯仰 °"), 1, 2);
    layout->addWidget(mount_pitch_, 1, 3);
    layout->addWidget(new QLabel("偏航 °"), 1, 4);
    layout->addWidget(mount_yaw_, 1, 5);
    layout->addWidget(apply_mount_button_, 2, 0, 1, 3);
    layout->addWidget(check_tf_button_, 2, 3, 1, 3);
    layout->setVerticalSpacing(1);
    layout->setHorizontalSpacing(2);
    group->setLayout(layout);
    return group;
  }

  QGroupBox* createSamplingGroup()
  {
    auto* group = new QGroupBox("滤波与稳定窗口");
    auto* form = new QFormLayout;
    form->setHorizontalSpacing(4);
    form->setVerticalSpacing(1);
    filter_enabled_check_ = new QCheckBox;
    filter_enabled_check_->setChecked(true);
    filter_window_spin_ = makeIntSpin(1, 200, 10);
    min_stable_samples_spin_ = makeIntSpin(1, 500, 20);
    max_stddev_force_spin_ = makeDoubleSpin(0.0001, 1000.0, 2.0);
    max_stddev_torque_spin_ = makeDoubleSpin(0.0001, 1000.0, 0.2);
    sample_timeout_spin_ = makeDoubleSpin(0.05, 30.0, 1.0, 3);
    apply_sampling_button_ = new QPushButton("应用滤波/采样");
    form->addRow("ROS滤波启用", filter_enabled_check_);
    form->addRow("移动平均窗口", filter_window_spin_);
    form->addRow("最小稳定样本", min_stable_samples_spin_);
    form->addRow("最大力标准差 N", max_stddev_force_spin_);
    form->addRow("最大力矩标准差 Nm", max_stddev_torque_spin_);
    form->addRow("采样超时 秒", sample_timeout_spin_);
    form->addRow(apply_sampling_button_);
    group->setLayout(form);
    return group;
  }

  QGroupBox* createSafetyGroup()
  {
    auto* group = new QGroupBox("安全检查清单");
    auto* layout = new QGridLayout;
    layout->setSpacing(1);
    layout->setVerticalSpacing(0);
    estop_check_ = new QCheckBox("急停按钮可达");
    teach_pendant_check_ = new QCheckBox("示教器由操作员持有");
    fence_check_ = new QCheckBox("安全区域已清空");
    low_speed_check_ = new QCheckBox("低速模式已激活");
    tool_fixed_check_ = new QCheckBox("工具和负载已固定");
    safety_label_ = new QLabel;
    safety_label_->setWordWrap(true);
    safety_label_->setStyleSheet("QLabel { color: #555; font-style: italic; font-size: 10px; }");
    layout->addWidget(estop_check_, 0, 0);
    layout->addWidget(teach_pendant_check_, 0, 1);
    layout->addWidget(fence_check_, 1, 0);
    layout->addWidget(low_speed_check_, 1, 1);
    layout->addWidget(tool_fixed_check_, 2, 0);
    layout->addWidget(safety_label_, 3, 0, 1, 2);
    group->setLayout(layout);
    return group;
  }

  QGroupBox* createAcquisitionGroup()
  {
    auto* group = new QGroupBox("负载辨识");
    auto* layout = new QHBoxLayout;
    layout->setSpacing(4);

    // Left: action buttons
    auto* btn_layout = new QVBoxLayout;
    btn_layout->setSpacing(2);
    auto* row1 = new QHBoxLayout;
    tool_button_ = new QPushButton("记录 仅工具");
    total_button_ = new QPushButton("记录 工具加负载");
    row1->addWidget(tool_button_);
    row1->addWidget(total_button_);
    auto* row2 = new QHBoxLayout;
    compute_button_ = new QPushButton("计算负载");
    clear_profiles_button_ = new QPushButton("清除配置");
    row2->addWidget(compute_button_);
    row2->addWidget(clear_profiles_button_);
    btn_layout->addLayout(row1);
    btn_layout->addLayout(row2);

    // Right: min samples
    auto* cfg_layout = new QFormLayout;
    cfg_layout->setSpacing(1);
    min_samples_spin_ = makeIntSpin(3, 100, 6);
    cfg_layout->addRow("最小样本数", min_samples_spin_);

    layout->addLayout(btn_layout, 1);
    layout->addLayout(cfg_layout);
    group->setLayout(layout);
    return group;
  }

  QWidget* createToolPositionBar()
  {
    auto* bar = new QWidget;
    bar->setStyleSheet("QWidget#toolBar { background: #f0f4f8; border-radius: 3px; }");
    bar->setObjectName("toolBar");
    auto* layout = new QHBoxLayout;
    layout->setContentsMargins(4, 1, 4, 1);
    layout->setSpacing(8);

    auto* label = new QLabel("工具位姿");
    label->setStyleSheet("QLabel { font-weight: bold; font-size: 10px; color: #333; border: none; background: transparent; }");
    tool_pos_bar_label_ = new QLabel("等待 TF...");
    tool_pos_bar_label_->setStyleSheet("QLabel { font-family: monospace; font-size: 10px; color: #555; border: none; background: transparent; }");
    tool_pos_bar_status_ = new QLabel("");
    tool_pos_bar_status_->setStyleSheet("QLabel { font-size: 9px; border: none; background: transparent; }");

    layout->addWidget(label);
    layout->addWidget(tool_pos_bar_label_, 1);
    layout->addWidget(tool_pos_bar_status_);
    bar->setLayout(layout);
    return bar;
  }

  void updateToolPosition()
  {
    try
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform("base_link", "tool0", ros::Time(0), transform);
      const double x = transform.getOrigin().x();
      const double y = transform.getOrigin().y();
      const double z = transform.getOrigin().z();
      double roll, pitch, yaw;
      transform.getBasis().getRPY(roll, pitch, yaw);

      last_tool_x_ = x; last_tool_y_ = y; last_tool_z_ = z;
      last_tool_roll_ = roll; last_tool_pitch_ = pitch; last_tool_yaw_ = yaw;

      tool_pos_bar_label_->setText(
        QString("X=%1 Y=%2 Z=%3 | R=%4° P=%5° Y=%6°")
          .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3)
          .arg(roll * 180.0 / M_PI, 0, 'f', 1)
          .arg(pitch * 180.0 / M_PI, 0, 'f', 1)
          .arg(yaw * 180.0 / M_PI, 0, 'f', 1));
      tool_pos_bar_status_->setText("✓ TF");
      tool_pos_bar_status_->setStyleSheet("QLabel { color: green; font-size: 9px; border: none; background: transparent; }");
    }
    catch (const tf::TransformException& ex)
    {
      (void)ex;
      tool_pos_bar_label_->setText("TF 缺失");
      tool_pos_bar_status_->setText("✗");
      tool_pos_bar_status_->setStyleSheet("QLabel { color: red; font-size: 9px; border: none; background: transparent; }");
    }
  }

  QGroupBox* createLogGroup()
  {
    auto* group = new QGroupBox("操作日志");
    auto* layout = new QVBoxLayout;
    layout->setSpacing(1);
    layout->setContentsMargins(2, 2, 2, 2);

    auto* header = new QHBoxLayout;
    auto* label = new QLabel("操作日志");
    label->setStyleSheet("QLabel { font-weight: bold; font-size: 10px; border: none; }");
    clear_log_button_ = new QPushButton("清除");
    clear_log_button_->setMaximumHeight(20);
    clear_log_button_->setStyleSheet("QPushButton { font-size: 9px; padding: 1px 6px; }");
    header->addWidget(label);
    header->addStretch();
    header->addWidget(clear_log_button_);

    log_view_ = new QPlainTextEdit;
    log_view_->setReadOnly(true);
    log_view_->setMaximumBlockCount(500);
    log_view_->setMaximumHeight(120);
    log_view_->setPlaceholderText("操作记录...");
    log_view_->setStyleSheet("QPlainTextEdit { font-family: monospace; font-size: 10px; }");

    layout->addLayout(header);
    layout->addWidget(log_view_);
    group->setLayout(layout);
    group->setMaximumHeight(160);
    return group;
  }

  void recordSample(uint8_t profile_type, const char* label)
  {
    // ── Safety: log current tool position for traceability ──
    std::ostringstream pose_log;
    pose_log << std::fixed << std::setprecision(4)
             << "当前 tool0 位姿: x=" << last_tool_x_ << " y=" << last_tool_y_
             << " z=" << last_tool_z_ << " roll=" << last_tool_roll_
             << " pitch=" << last_tool_pitch_ << " yaw=" << last_tool_yaw_;
    logOperation(QString::fromStdString(pose_log.str()));

    StepControl srv;
    srv.request.profile_type = profile_type;
    srv.request.step_index = next_step_index_++;
    srv.request.execute_motion = false;
    srv.request.sample_wrench = true;
    srv.request.manual_confirmed = true;
    srv.request.target_pose_name = label;

    if (!step_client_.waitForExistence(ros::Duration(0.2)) || !step_client_.call(srv))
    {
      status_label_->setText("step_control 服务不可用。");
      logOperation("✗ step_control 服务不可用");
      return;
    }

    std::ostringstream status;
    status << label << ": " << srv.response.message
           << " (已采集样本=" << srv.response.samples_collected << ")";
    QString msg = QString::fromStdString(status.str());
    status_label_->setText(msg);
    logOperation(srv.response.accepted ? "✓ " : "✗ " + msg);

    // ── Safety: warn if sample count is low ──
    if (srv.response.accepted && srv.response.samples_collected < 6)
    {
      logOperation("⚠ 建议至少采集 6 个不同姿态的样本以获得可靠的辨识结果");
    }
  }

  // ═══════════════════════════════════════════════════════════════
  //  Members
  // ═══════════════════════════════════════════════════════════════

  ros::NodeHandle nh_;
  ros::ServiceClient step_client_;
  ros::ServiceClient compute_client_;
  ros::ServiceClient sensor_mount_client_;
  ros::ServiceClient sampling_config_client_;
  ros::ServiceClient sri_filter_client_;
  ros::Subscriber wrench_monitor_sub_;
  tf::TransformListener tf_listener_;
  QTimer* refresh_timer_{nullptr};

  QLabel* status_label_{nullptr};
  QLabel* safety_label_{nullptr};
  QLabel* sensor_status_label_{nullptr};
  QLabel* robot_status_label_{nullptr};
  QComboBox* mode_combo_{nullptr};
  QLineEdit* robot_ip_edit_{nullptr};
  QLineEdit* sensor_ip_edit_{nullptr};
  QSpinBox* eki_port_spin_{nullptr};
  QSpinBox* publish_rate_spin_{nullptr};
  QPushButton* sensor_connect_button_{nullptr};
  QPushButton* robot_connect_button_{nullptr};
  ros::Time last_wrench_stamp_{0.0};
  QDoubleSpinBox* mount_x_{nullptr};
  QDoubleSpinBox* mount_y_{nullptr};
  QDoubleSpinBox* mount_z_{nullptr};
  QDoubleSpinBox* mount_roll_{nullptr};
  QDoubleSpinBox* mount_pitch_{nullptr};
  QDoubleSpinBox* mount_yaw_{nullptr};
  QPushButton* apply_mount_button_{nullptr};
  QPushButton* check_tf_button_{nullptr};
  QCheckBox* filter_enabled_check_{nullptr};
  QSpinBox* filter_window_spin_{nullptr};
  QSpinBox* min_stable_samples_spin_{nullptr};
  QDoubleSpinBox* max_stddev_force_spin_{nullptr};
  QDoubleSpinBox* max_stddev_torque_spin_{nullptr};
  QDoubleSpinBox* sample_timeout_spin_{nullptr};
  QPushButton* apply_sampling_button_{nullptr};
  QCheckBox* estop_check_{nullptr};
  QCheckBox* teach_pendant_check_{nullptr};
  QCheckBox* fence_check_{nullptr};
  QCheckBox* low_speed_check_{nullptr};
  QCheckBox* tool_fixed_check_{nullptr};
  QPushButton* tool_button_{nullptr};
  QPushButton* total_button_{nullptr};
  QPushButton* compute_button_{nullptr};
  QPushButton* clear_profiles_button_{nullptr};
  QSpinBox* min_samples_spin_{nullptr};
  QPlainTextEdit* log_view_{nullptr};
  QPushButton* clear_log_button_{nullptr};
  QLabel* tool_pos_bar_label_{nullptr};
  QLabel* tool_pos_bar_status_{nullptr};
  double last_tool_x_{0.0};
  double last_tool_y_{0.0};
  double last_tool_z_{0.0};
  double last_tool_roll_{0.0};
  double last_tool_pitch_{0.0};
  double last_tool_yaw_{0.0};
  uint32_t next_step_index_{0};
};
}  // namespace tool_gravity_compensation

PLUGINLIB_EXPORT_CLASS(tool_gravity_compensation::ToolGravityPanel, rviz::Panel)

#include "tool_gravity_panel.moc"
