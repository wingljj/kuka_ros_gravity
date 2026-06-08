#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
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
#include <QProcess>
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

    main_layout->addWidget(createHeader());
    main_layout->addWidget(createToolPositionBar());

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

    main_layout->addWidget(createAcquisitionGroup());

    status_label_ = new QLabel("就绪。需要手动确认。");
    status_label_->setWordWrap(true);
    status_label_->setStyleSheet("QLabel { color: #333; font-weight: bold; padding: 1px 2px; font-size: 11px; }");
    main_layout->addWidget(status_label_);

    main_layout->addWidget(createLogGroup());
    setLayout(main_layout);

    // Service clients
    step_client_ = nh_.serviceClient<StepControl>("step_control");
    compute_client_ = nh_.serviceClient<ComputePayload>("compute_payload");
    sensor_mount_client_ = nh_.serviceClient<SetSensorMount>("set_sensor_mount");
    sampling_config_client_ = nh_.serviceClient<SetSamplingConfig>("set_sampling_config");
    sri_filter_client_ = nh_.serviceClient<sriforcesensor::SetFilterConfig>("sri_ft_sensor/set_filter_config");

    // Data monitors
    wrench_sub_ = nh_.subscribe("/sri_ft_sensor/wrench", 10, &ToolGravityPanel::handleWrench, this);
    joint_sub_ = nh_.subscribe("/joint_states", 10, &ToolGravityPanel::handleJoint, this);

    // Signal/slot
    connect(apply_mount_button_, SIGNAL(clicked()), this, SLOT(onApplyMountClicked()));
    connect(check_tf_button_, SIGNAL(clicked()), this, SLOT(onCheckTfClicked()));
    connect(apply_sampling_button_, SIGNAL(clicked()), this, SLOT(onApplySamplingClicked()));
    connect(tool_button_, SIGNAL(clicked()), this, SLOT(onToolOnlyClicked()));
    connect(total_button_, SIGNAL(clicked()), this, SLOT(onToolPlusPayloadClicked()));
    connect(compute_button_, SIGNAL(clicked()), this, SLOT(onComputeClicked()));
    connect(clear_profiles_button_, SIGNAL(clicked()), this, SLOT(onClearProfilesClicked()));
    connect(sensor_btn_, SIGNAL(clicked()), this, SLOT(onSensorConnect()));
    connect(robot_btn_, SIGNAL(clicked()), this, SLOT(onRobotConnect()));
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
    onDetectStatus();
  }

  ~ToolGravityPanel() override
  {
    if (sensor_proc_ && sensor_proc_->state() != QProcess::NotRunning)
    { sensor_proc_->terminate(); sensor_proc_->waitForFinished(2000); }
    if (robot_proc_ && robot_proc_->state() != QProcess::NotRunning)
    { robot_proc_->terminate(); robot_proc_->waitForFinished(3000); }
  }

private Q_SLOTS:
  // ═══ 连接管理 + 状态检测 ═══
  void updateButtons()
  {
    bool s_alive = sensor_proc_ && sensor_proc_->state() == QProcess::Running;
    bool r_alive = robot_proc_ && robot_proc_->state() == QProcess::Running;
    sensor_btn_->setText(s_alive ? "断开传感器" : "连接传感器");
    sensor_btn_->setStyleSheet(s_alive ? "QPushButton{font-weight:bold;padding:3px 6px;background:#f44336;color:white;}" : "QPushButton{font-weight:bold;padding:3px 6px;}");
    robot_btn_->setText(r_alive ? "断开机器人" : "连接机器人");
    robot_btn_->setStyleSheet(r_alive ? "QPushButton{font-weight:bold;padding:3px 6px;background:#f44336;color:white;}" : "QPushButton{font-weight:bold;padding:3px 6px;}");
  }

  void onSensorConnect()
  {
    if (sensor_proc_ && sensor_proc_->state() == QProcess::Running)
    { disconnectSensor(); return; }

    const QString ip = sensor_ip_edit_->text();
    const int rate = publish_rate_spin_->value();
    logOperation("▶ 启动传感器 " + ip + " @" + QString::number(rate) + "Hz");
    sensor_status_->setText("● 启动中...");

    sensor_proc_ = new QProcess(this);
    QStringList args; args << "sriforcesensor" << "forcesensor" << ip << QString::number(rate);
    connect(sensor_proc_, QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),
      this, [this](int c, QProcess::ExitStatus){ if(c!=0){ logOperation("✗ 传感器退出(code="+QString::number(c)+")，无硬件"); sensor_status_->setText("○ 无硬件"); } updateButtons(); });
    connect(sensor_proc_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError){ sensor_status_->setText("○ 启动失败"); updateButtons(); });
    sensor_proc_->setProcessChannelMode(QProcess::MergedChannels);
    sensor_proc_->start("rosrun", args);
    updateButtons();
  }

  void onRobotConnect()
  {
    if (robot_proc_ && robot_proc_->state() == QProcess::Running)
    { disconnectRobot(); return; }

    const QString ip = robot_ip_edit_->text();
    const int port = eki_port_spin_->value();
    logOperation("▶ 启动 EKI 接口 " + ip + ":" + QString::number(port));
    robot_status_->setText("● 启动中...");

    nh_.setParam("/eki/robot_address", ip.toStdString());
    nh_.setParam("/eki/robot_port", port);
    nh_.setParam("/robot_ip_address", ip.toStdString());

    robot_proc_ = new QProcess(this);
    QStringList args; args << "kuka_eki_hw_interface" << "kuka_eki_hw_interface_node";
    connect(robot_proc_, QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),
      this, [this](int c, QProcess::ExitStatus){ if(c!=0){ logOperation("✗ EKI退出(code="+QString::number(c)+")，无硬件"); robot_status_->setText("○ 无硬件"); } updateButtons(); });
    connect(robot_proc_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError){ robot_status_->setText("○ 启动失败"); updateButtons(); });
    robot_proc_->setProcessChannelMode(QProcess::MergedChannels);
    robot_proc_->start("rosrun", args);
    updateButtons();
  }

  void disconnectSensor()
  {
    if (sensor_proc_) { sensor_proc_->terminate(); sensor_proc_->waitForFinished(2000); sensor_proc_->deleteLater(); sensor_proc_=nullptr; }
    last_wrench_time_ = 0;
    sensor_status_->setText("○ 已断开"); logOperation("✓ 传感器已断开"); updateButtons();
  }

  void disconnectRobot()
  {
    if (robot_proc_) { robot_proc_->terminate(); robot_proc_->waitForFinished(3000); robot_proc_->deleteLater(); robot_proc_=nullptr; }
    last_joint_time_ = 0;
    robot_status_->setText("○ 已断开"); logOperation("✓ 机器人已断开"); updateButtons();
  }

  void onDetectStatus()
  {
    updateToolPosition();
    refreshStatus(true);
  }

  void onRefreshStatusClicked()
  {
    updateToolPosition();
    refreshStatus(false);
  }

  void refreshStatus(bool verbose)
  {
    const double now = ros::Time::now().toSec();
    const double w_age = now - last_wrench_time_;
    const double j_age = now - last_joint_time_;
    const bool w_ok = (last_wrench_time_ > 0 && w_age < 2.0);
    const bool j_ok = (last_joint_time_ > 0 && j_age < 2.0);

    // Sensor: raw_wrench topic == real SRI hardware
    bool raw_ok = false;
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics))
      for (const auto& t : topics)
        if (t.name == "/sri_ft_sensor/raw_wrench") { raw_ok = true; break; }

    // Robot: kuka_eki node MUST exist (fake controller doesn't need it)
    bool eki_ok = false;
    ros::V_string nodes;
    if (ros::master::getNodes(nodes))
      for (const auto& n : nodes)
        if (n.find("kuka_eki") != std::string::npos) { eki_ok = true; break; }

    if (w_ok && raw_ok)
      sensor_status_->setText(QString("● 硬件已连接  %1 Hz").arg(1.0/std::max(w_age,0.01),0,'f',0));
    else if (w_ok && !raw_ok)
      sensor_status_->setText("● 仿真数据 (离线模式)");
    else
      sensor_status_->setText("○ 无数据");

    if (j_ok && eki_ok)
      robot_status_->setText("● 硬件已连接");
    else if (j_ok && !eki_ok)
      robot_status_->setText("● 仿真 (无EKI节点)");
    else
      robot_status_->setText("○ 未连接");

    if (verbose)
      logOperation("状态: 传感器=" + sensor_status_->text() + "  机器人=" + robot_status_->text());
  }

  void handleWrench(const geometry_msgs::WrenchStampedConstPtr& msg)
  { last_wrench_time_ = msg->header.stamp.toSec(); }

  void handleJoint(const sensor_msgs::JointStateConstPtr& msg)
  { last_joint_time_ = msg->header.stamp.toSec(); }

  // ═══ 传感器安装 ═══
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
    { status_label_->setText("set_sensor_mount 服务不可用。"); logOperation("✗ set_sensor_mount 服务不可用"); return; }
    QString msg = QString::fromStdString(srv.response.message);
    status_label_->setText(msg);
    logOperation("✓ " + msg);
  }

  void onCheckTfClicked()
  {
    logOperation("▶ 检查 TF tool0 → sri_ft_sensor...");
    tf::StampedTransform t;
    try
    {
      tf_listener_.lookupTransform("tool0", "sri_ft_sensor", ros::Time(0), t);
      std::ostringstream s; s << std::fixed << std::setprecision(4)
        << "TF正常 tool0→sri_ft_sensor xyz=[" << t.getOrigin().x() << "," << t.getOrigin().y() << "," << t.getOrigin().z() << "]";
      status_label_->setText(QString::fromStdString(s.str()));
      logOperation("✓ " + QString::fromStdString(s.str()));
    }
    catch (const tf::TransformException& ex)
    { status_label_->setText(QString("TF缺失: ") + ex.what()); logOperation("✗ TF缺失: " + QString(ex.what())); }
  }

  // ═══ 滤波采样 ═══
  void onApplySamplingClicked()
  {
    logOperation("▶ 应用滤波/采样配置...");
    SetSamplingConfig srv;
    srv.request.apply = true; srv.request.clear_profiles = false;
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

    bool sri_ok = false;
    sriforcesensor::SetFilterConfig fs;
    fs.request.apply = true; fs.request.enabled = filter_enabled_check_->isChecked();
    fs.request.window_size = filter_window_spin_->value();
    fs.request.max_stddev_force = max_stddev_force_spin_->value();
    fs.request.max_stddev_torque = max_stddev_torque_spin_->value();
    if (sri_filter_client_.waitForExistence(ros::Duration(0.2)))
      sri_ok = sri_filter_client_.call(fs) && fs.response.success;

    if (!sampling_config_client_.waitForExistence(ros::Duration(0.2)) || !sampling_config_client_.call(srv))
    { status_label_->setText("set_sampling_config 服务不可用。"); logOperation("✗ set_sampling_config 服务不可用"); return; }
    std::ostringstream st; st << srv.response.message;
    if (mode_combo_->currentIndex() == 0) st << " | 离线模式";
    else if (sri_ok) st << " | " << fs.response.message;
    else st << " | SRI滤波服务缺失";
    status_label_->setText(QString::fromStdString(st.str()));
    logOperation("✓ " + QString::fromStdString(st.str()));
  }

  // ═══ 负载辨识 ═══
  void onToolOnlyClicked()
  { logOperation("▶ 记录 TOOL_ONLY..."); recordSample(StepControl::Request::TOOL_ONLY, "TOOL_ONLY"); }
  void onToolPlusPayloadClicked()
  { logOperation("▶ 记录 TOOL_PLUS_PAYLOAD..."); recordSample(StepControl::Request::TOOL_PLUS_PAYLOAD, "TOOL_PLUS_PAYLOAD"); }

  void onComputeClicked()
  {
    logOperation("▶ 计算负载参数...");
    ComputePayload srv; srv.request.compute = true;
    srv.request.min_samples = static_cast<uint32_t>(min_samples_spin_->value());
    if (!compute_client_.waitForExistence(ros::Duration(0.2)) || !compute_client_.call(srv))
    { status_label_->setText("compute_payload 服务不可用。"); logOperation("✗ compute_payload 服务不可用"); return; }
    status_label_->setText(QString::fromStdString(srv.response.message));
    if (!srv.response.success) { logOperation("✗ " + QString::fromStdString(srv.response.message)); return; }
    std::ostringstream r; r << std::fixed << std::setprecision(4)
      << "负载质量:" << srv.response.result.mass_kg << " kg\n"
      << "负载重量:" << srv.response.result.weight_n << " N\n"
      << "负载质心(sri_ft_sensor系):[" << srv.response.result.com_sensor.x << ","
      << srv.response.result.com_sensor.y << "," << srv.response.result.com_sensor.z << "] m\n"
      << "工具质量:" << srv.response.result.tool_mass << " kg";
    status_label_->setText("✓ 负载计算完成");
    logOperation("✓ 负载计算完成:\n" + QString::fromStdString(r.str()));
    ROS_INFO_STREAM("负载结果: " << r.str());
  }

  void onClearProfilesClicked()
  {
    logOperation("▶ 清除采集配置...");
    SetSamplingConfig srv; srv.request.apply = true; srv.request.clear_profiles = true;
    srv.request.filter_window_size = filter_window_spin_->value();
    srv.request.min_stable_samples = min_stable_samples_spin_->value();
    srv.request.max_stddev_force = max_stddev_force_spin_->value();
    srv.request.max_stddev_torque = max_stddev_torque_spin_->value();
    srv.request.sample_timeout = sample_timeout_spin_->value();
    if (!sampling_config_client_.waitForExistence(ros::Duration(0.2)) || !sampling_config_client_.call(srv))
    { status_label_->setText("set_sampling_config 服务不可用。"); logOperation("✗ set_sampling_config 服务不可用"); return; }
    next_step_index_ = 0;
    status_label_->setText(QString::fromStdString(srv.response.message));
    logOperation("✓ 配置已清除");
  }

  void onClearLogClicked() { log_view_->clear(); logOperation("日志已清除"); }

  void updateSafetyGate()
  {
    const bool safe = estop_check_->isChecked() && teach_pendant_check_->isChecked() &&
                      fence_check_->isChecked() && low_speed_check_->isChecked() && tool_fixed_check_->isChecked();
    const bool was = tool_button_->isEnabled();
    tool_button_->setEnabled(safe); total_button_->setEnabled(safe); compute_button_->setEnabled(safe);
    safety_label_->setText(safe ? "安全检查已完成。" : "采样前请完成安全检查。");
    if (safe != was)
      logOperation(safe ? "✓ 安全检查全部通过，采样已启用" : "⚠ 安全检查未完成，采样已禁用");
  }

private:
  void logOperation(const QString& msg)
  {
    log_view_->appendPlainText("[" + QDateTime::currentDateTime().toString("hh:mm:ss") + "] " + msg);
    log_view_->verticalScrollBar()->setValue(log_view_->verticalScrollBar()->maximum());
  }

  void updateToolPosition()
  {
    try
    {
      tf::StampedTransform t;
      tf_listener_.lookupTransform("base_link", "tool0", ros::Time(0), t);
      double r, p, y; t.getBasis().getRPY(r, p, y);
      last_tool_x_ = t.getOrigin().x(); last_tool_y_ = t.getOrigin().y(); last_tool_z_ = t.getOrigin().z();
      tool_pos_bar_->setText(QString("X=%1 Y=%2 Z=%3 | R=%4° P=%5° Y=%6°")
        .arg(last_tool_x_, 0, 'f', 3).arg(last_tool_y_, 0, 'f', 3).arg(last_tool_z_, 0, 'f', 3)
        .arg(r * 180 / M_PI, 0, 'f', 1).arg(p * 180 / M_PI, 0, 'f', 1).arg(y * 180 / M_PI, 0, 'f', 1));
      tool_pos_status_->setText("✓ TF"); tool_pos_status_->setStyleSheet("QLabel{color:green;font-size:9px;border:none;background:transparent;}");
    }
    catch (const tf::TransformException&)
    { tool_pos_bar_->setText("TF 缺失"); tool_pos_status_->setText("✗"); tool_pos_status_->setStyleSheet("QLabel{color:red;font-size:9px;border:none;background:transparent;}"); }
  }

  void recordSample(uint8_t profile_type, const char* label)
  {
    std::ostringstream pl; pl << std::fixed << std::setprecision(4)
      << "当前tool0位姿: x=" << last_tool_x_ << " y=" << last_tool_y_ << " z=" << last_tool_z_;
    logOperation(QString::fromStdString(pl.str()));
    StepControl srv; srv.request.profile_type = profile_type; srv.request.step_index = next_step_index_++;
    srv.request.execute_motion = false; srv.request.sample_wrench = true;
    srv.request.manual_confirmed = true; srv.request.target_pose_name = label;
    if (!step_client_.waitForExistence(ros::Duration(0.2)) || !step_client_.call(srv))
    { status_label_->setText("step_control 服务不可用。"); logOperation("✗ step_control 服务不可用"); return; }
    std::ostringstream st; st << label << ": " << srv.response.message << " (已采集=" << srv.response.samples_collected << ")";
    status_label_->setText(QString::fromStdString(st.str()));
    logOperation(srv.response.accepted ? "✓ " : "✗ " + QString::fromStdString(st.str()));
    if (srv.response.accepted && srv.response.samples_collected < 6)
      logOperation("⚠ 建议至少6个不同姿态");
  }

  // ═══ UI builders ═══

  QWidget* createHeader()
  {
    auto* w = new QWidget;
    w->setStyleSheet("QWidget#header{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0#0a1f44,stop:0.5#1a3a6e,stop:1#0a1f44);border-radius:4px;}");
    w->setObjectName("header");
    auto* l = new QHBoxLayout; l->setContentsMargins(4,2,4,2); l->setSpacing(6);
    auto* logo = new QLabel("🚀 中国航天");
    logo->setStyleSheet("QLabel{font-size:12px;font-weight:bold;color:#d4e4ff;background:#0d2b5e;border:1px solid #3a7bd5;border-radius:3px;padding:2px 5px;}");
    auto* title = new QLabel("上海卫星装备研究所 · 机械臂负载测试系统");
    title->setStyleSheet("QLabel{font-size:13px;font-weight:bold;color:#e8efff;border:none;background:transparent;}");
    l->addWidget(logo); l->addWidget(title,1); w->setLayout(l); return w;
  }

  QWidget* createToolPositionBar()
  {
    auto* w = new QWidget;
    w->setStyleSheet("QWidget#toolBar{background:#f0f4f8;border-radius:3px;}"); w->setObjectName("toolBar");
    auto* l = new QHBoxLayout; l->setContentsMargins(4,1,4,1); l->setSpacing(8);
    auto* lb = new QLabel("工具位姿"); lb->setStyleSheet("QLabel{font-weight:bold;font-size:10px;color:#333;border:none;background:transparent;}");
    tool_pos_bar_ = new QLabel("等待 TF...");
    tool_pos_bar_->setStyleSheet("QLabel{font-family:monospace;font-size:10px;color:#555;border:none;background:transparent;}");
    tool_pos_status_ = new QLabel("");
    l->addWidget(lb); l->addWidget(tool_pos_bar_,1); l->addWidget(tool_pos_status_); w->setLayout(l); return w;
  }

  QGroupBox* createConnectionGroup()
  {
    auto* g = new QGroupBox("设备状态");
    auto* lay = new QVBoxLayout; lay->setSpacing(2);

    auto* row1 = new QHBoxLayout;
    row1->addWidget(new QLabel("模式"));
    mode_combo_ = new QComboBox;
    mode_combo_->addItem("离线仿真"); mode_combo_->addItem("真实传感器"); mode_combo_->addItem("真实机器人");
    row1->addWidget(mode_combo_, 1);
    lay->addLayout(row1);

    auto* srow = new QHBoxLayout;
    sensor_btn_ = new QPushButton("连接传感器"); sensor_btn_->setStyleSheet("QPushButton{font-weight:bold;padding:3px 6px;}");
    sensor_status_ = new QLabel("○ 检测中..."); sensor_status_->setStyleSheet("QLabel{font-size:11px;font-weight:bold;padding:2px;}");
    srow->addWidget(sensor_btn_); srow->addWidget(sensor_status_, 1);
    lay->addLayout(srow);

    auto* rrow = new QHBoxLayout;
    robot_btn_ = new QPushButton("连接机器人"); robot_btn_->setStyleSheet("QPushButton{font-weight:bold;padding:3px 6px;}");
    robot_status_ = new QLabel("○ 检测中..."); robot_status_->setStyleSheet("QLabel{font-size:11px;font-weight:bold;padding:2px;}");
    rrow->addWidget(robot_btn_); rrow->addWidget(robot_status_, 1);
    lay->addLayout(rrow);

    auto* row2 = new QHBoxLayout;
    row2->addWidget(new QLabel("SRI IP")); sensor_ip_edit_ = new QLineEdit("192.168.0.108"); sensor_ip_edit_->setMaximumWidth(110); row2->addWidget(sensor_ip_edit_);
    row2->addWidget(new QLabel("机器人IP")); robot_ip_edit_ = new QLineEdit("172.31.1.147"); robot_ip_edit_->setMaximumWidth(110); row2->addWidget(robot_ip_edit_);
    row2->addWidget(new QLabel("端口")); eki_port_spin_ = makeIntSpin(1,65535,54600); eki_port_spin_->setMaximumWidth(65); row2->addWidget(eki_port_spin_);
    row2->addWidget(new QLabel("Hz")); publish_rate_spin_ = makeIntSpin(1,1000,200); publish_rate_spin_->setMaximumWidth(60); row2->addWidget(publish_rate_spin_);
    lay->addLayout(row2);

    g->setLayout(lay); return g;
  }

  QGroupBox* createMountGroup()
  {
    auto* g = new QGroupBox("传感器安装 tool0 → sri_ft_sensor");
    auto* lay = new QGridLayout; lay->setVerticalSpacing(1); lay->setHorizontalSpacing(2);
    mount_x_ = makeDoubleSpin(-5,5,0); mount_y_ = makeDoubleSpin(-5,5,0); mount_z_ = makeDoubleSpin(-5,5,0);
    mount_roll_ = makeDoubleSpin(-180,180,0,1); mount_pitch_ = makeDoubleSpin(-180,180,0,1); mount_yaw_ = makeDoubleSpin(-180,180,0,1);
    apply_mount_button_ = new QPushButton("应用安装"); check_tf_button_ = new QPushButton("检查TF");
    lay->addWidget(new QLabel("x 米"),0,0); lay->addWidget(mount_x_,0,1);
    lay->addWidget(new QLabel("y 米"),0,2); lay->addWidget(mount_y_,0,3);
    lay->addWidget(new QLabel("z 米"),0,4); lay->addWidget(mount_z_,0,5);
    lay->addWidget(new QLabel("滚转°"),1,0); lay->addWidget(mount_roll_,1,1);
    lay->addWidget(new QLabel("俯仰°"),1,2); lay->addWidget(mount_pitch_,1,3);
    lay->addWidget(new QLabel("偏航°"),1,4); lay->addWidget(mount_yaw_,1,5);
    lay->addWidget(apply_mount_button_,2,0,1,3); lay->addWidget(check_tf_button_,2,3,1,3);
    g->setLayout(lay); return g;
  }

  QGroupBox* createSamplingGroup()
  {
    auto* g = new QGroupBox("滤波与稳定窗口");
    auto* f = new QFormLayout; f->setHorizontalSpacing(4); f->setVerticalSpacing(1);
    filter_enabled_check_ = new QCheckBox; filter_enabled_check_->setChecked(true);
    filter_window_spin_ = makeIntSpin(1,200,10);
    min_stable_samples_spin_ = makeIntSpin(1,500,20);
    max_stddev_force_spin_ = makeDoubleSpin(0.0001,1000,2.0);
    max_stddev_torque_spin_ = makeDoubleSpin(0.0001,1000,0.2);
    sample_timeout_spin_ = makeDoubleSpin(0.05,30,1.0,3);
    apply_sampling_button_ = new QPushButton("应用滤波/采样");
    f->addRow("ROS滤波启用",filter_enabled_check_); f->addRow("移动平均窗口",filter_window_spin_);
    f->addRow("最小稳定样本",min_stable_samples_spin_); f->addRow("最大力标准差 N",max_stddev_force_spin_);
    f->addRow("最大力矩标准差 Nm",max_stddev_torque_spin_); f->addRow("采样超时 秒",sample_timeout_spin_);
    f->addRow(apply_sampling_button_); g->setLayout(f); return g;
  }

  QGroupBox* createSafetyGroup()
  {
    auto* g = new QGroupBox("安全检查清单");
    auto* lay = new QGridLayout; lay->setSpacing(1); lay->setVerticalSpacing(0);
    estop_check_ = new QCheckBox("急停按钮可达"); teach_pendant_check_ = new QCheckBox("示教器由操作员持有");
    fence_check_ = new QCheckBox("安全区域已清空"); low_speed_check_ = new QCheckBox("低速模式已激活");
    tool_fixed_check_ = new QCheckBox("工具和负载已固定");
    safety_label_ = new QLabel; safety_label_->setWordWrap(true);
    safety_label_->setStyleSheet("QLabel{color:#555;font-style:italic;font-size:10px;}");
    lay->addWidget(estop_check_,0,0); lay->addWidget(teach_pendant_check_,0,1);
    lay->addWidget(fence_check_,1,0); lay->addWidget(low_speed_check_,1,1);
    lay->addWidget(tool_fixed_check_,2,0); lay->addWidget(safety_label_,3,0,1,2);
    g->setLayout(lay); return g;
  }

  QGroupBox* createAcquisitionGroup()
  {
    auto* g = new QGroupBox("负载辨识");
    auto* lay = new QHBoxLayout; lay->setSpacing(4);
    auto* bl = new QVBoxLayout; bl->setSpacing(2);
    auto* r1 = new QHBoxLayout; auto* r2 = new QHBoxLayout;
    tool_button_ = new QPushButton("记录 仅工具"); total_button_ = new QPushButton("记录 工具加负载");
    compute_button_ = new QPushButton("计算负载"); clear_profiles_button_ = new QPushButton("清除配置");
    r1->addWidget(tool_button_); r1->addWidget(total_button_);
    r2->addWidget(compute_button_); r2->addWidget(clear_profiles_button_);
    bl->addLayout(r1); bl->addLayout(r2);
    auto* cl = new QFormLayout; cl->setSpacing(1);
    min_samples_spin_ = makeIntSpin(3,100,6); cl->addRow("最小样本数",min_samples_spin_);
    lay->addLayout(bl,1); lay->addLayout(cl); g->setLayout(lay); return g;
  }

  QGroupBox* createLogGroup()
  {
    auto* g = new QGroupBox("操作日志"); auto* lay = new QVBoxLayout; lay->setSpacing(1); lay->setContentsMargins(2,2,2,2);
    auto* hdr = new QHBoxLayout;
    auto* lb = new QLabel("操作日志"); lb->setStyleSheet("QLabel{font-weight:bold;font-size:10px;border:none;}");
    clear_log_button_ = new QPushButton("清除"); clear_log_button_->setMaximumHeight(20);
    clear_log_button_->setStyleSheet("QPushButton{font-size:9px;padding:1px 6px;}");
    hdr->addWidget(lb); hdr->addStretch(); hdr->addWidget(clear_log_button_);
    log_view_ = new QPlainTextEdit; log_view_->setReadOnly(true); log_view_->setMaximumBlockCount(500);
    log_view_->setMaximumHeight(120); log_view_->setPlaceholderText("操作记录...");
    log_view_->setStyleSheet("QPlainTextEdit{font-family:monospace;font-size:10px;}");
    lay->addLayout(hdr); lay->addWidget(log_view_); g->setLayout(lay); g->setMaximumHeight(160); return g;
  }

  // ═══ Members ═══
  ros::NodeHandle nh_;
  ros::ServiceClient step_client_, compute_client_, sensor_mount_client_, sampling_config_client_, sri_filter_client_;
  ros::Subscriber wrench_sub_, joint_sub_;
  tf::TransformListener tf_listener_;
  QTimer* refresh_timer_{nullptr};

  QLabel *status_label_{nullptr}, *safety_label_{nullptr}, *sensor_status_{nullptr}, *robot_status_{nullptr};
  QLabel *tool_pos_bar_{nullptr}, *tool_pos_status_{nullptr};
  QComboBox* mode_combo_{nullptr};
  QLineEdit *robot_ip_edit_{nullptr}, *sensor_ip_edit_{nullptr};
  QSpinBox *eki_port_spin_{nullptr}, *publish_rate_spin_{nullptr};
  QPushButton *sensor_btn_{nullptr}, *robot_btn_{nullptr};
  QProcess *sensor_proc_{nullptr}, *robot_proc_{nullptr};
  QDoubleSpinBox *mount_x_{nullptr}, *mount_y_{nullptr}, *mount_z_{nullptr}, *mount_roll_{nullptr}, *mount_pitch_{nullptr}, *mount_yaw_{nullptr};
  QPushButton *apply_mount_button_{nullptr}, *check_tf_button_{nullptr};
  QCheckBox *filter_enabled_check_{nullptr};
  QSpinBox *filter_window_spin_{nullptr}, *min_stable_samples_spin_{nullptr};
  QDoubleSpinBox *max_stddev_force_spin_{nullptr}, *max_stddev_torque_spin_{nullptr}, *sample_timeout_spin_{nullptr};
  QPushButton *apply_sampling_button_{nullptr};
  QCheckBox *estop_check_{nullptr}, *teach_pendant_check_{nullptr}, *fence_check_{nullptr}, *low_speed_check_{nullptr}, *tool_fixed_check_{nullptr};
  QPushButton *tool_button_{nullptr}, *total_button_{nullptr}, *compute_button_{nullptr}, *clear_profiles_button_{nullptr};
  QSpinBox* min_samples_spin_{nullptr};
  QPlainTextEdit* log_view_{nullptr};
  QPushButton* clear_log_button_{nullptr};
  double last_wrench_time_{0}, last_joint_time_{0};
  double last_tool_x_{0}, last_tool_y_{0}, last_tool_z_{0};
  uint32_t next_step_index_{0};
};
}  // namespace tool_gravity_compensation

PLUGINLIB_EXPORT_CLASS(tool_gravity_compensation::ToolGravityPanel, rviz::Panel)
#include "tool_gravity_panel.moc"
