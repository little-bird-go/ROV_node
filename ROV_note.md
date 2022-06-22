### `ARDUPILOT`/`ARDUSUB`学习记录

#### 源码下载及环境配置步骤

1. 下载`git`工具，一般情况下`ubuntu`系统会自带
2. 克隆源码

```bash
git clone https://github.com/Ardupilot/ardupilot.git
```

3. 下载源码依赖包

```bash
git submodule update --init --recursive
```

4. 下载必要的工具

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

5. 设置环境变量

```bash
. ~/.profile
```

##### 注意事项

下载过程会因为网络原因而中断出错。出错后重复命令即可。如果条件允许可以挂个梯子。

#### 程序编译和烧录

1. 在首次编译时先对板子类型进行配置

```bash
./waf configure --board=Pixhawk1
```

2. 编译对应的程序

```bash
./waf sub		// 几种特定的车辆类型可以这么编译
./waf build --target=examples/UART_test
```

3. 烧录

```bash
./waf --upload sub
```

更多关于`waf`的使用，可以使用以下命令来查看

```bash
./waf --help
```

#### 主程序学习

##### `main`函数

```cpp
// 在 ArduSub.cpp 最后一行
AP_HAL_MAIN_CALLBACKS(&sub);

// 定义在 AP_HAL_Main.h 中
#define AP_HAL_MAIN_CALLBACKS(CALLBACKS) extern "C" { \
    int AP_MAIN(int argc, char* const argv[]); \
    int AP_MAIN(int argc, char* const argv[]) { \
        hal.run(argc, argv, CALLBACKS); \
        return 0; \
    } \
    }

// 即在 main 中调用了 hal.run() 函数
```

##### `hal.run()`函数

该函数的实现与使用的操作系统相关，不同的操作系统下有不同的实现。`ardupilot`之前使用的是`NUTTX`操作系统，但是之后全部移植到了`ChibiOS`操作系统下，这里以`ChibiOS`操作系统下为例。

```cpp
// 在 HAL_ChibiOS_Class.cpp 中

void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    /*
     * System initializations.
     * - ChibiOS HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */

#if HAL_USE_SERIAL_USB == TRUE
    usb_initialise();
#endif

#ifdef HAL_STDOUT_SERIAL
    //STDOUT Initialisation
    SerialConfig stdoutcfg =
    {
      HAL_STDOUT_BAUDRATE,
      0,
      USART_CR2_STOP1_BITS,
      0
    };
    sdStart((SerialDriver*)&HAL_STDOUT_SERIAL, &stdoutcfg);
#endif

    g_callbacks = callbacks;

    //Takeover main
    main_loop();
}

// 通过代码注释可知，在此函数中实现了一些驱动和内核的初始化
// 最后执行了 main_loop() 函数
```

##### `main_loop()`函数

```cpp
// 在 HAL_ChibiOS_Class.cpp 中

static void main_loop()
{
    daemon_task = chThdGetSelfX();

    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

#ifdef HAL_I2C_CLEAR_BUS
    // Clear all I2C Buses. This can be needed on some boards which
    // can get a stuck I2C peripheral on boot
    ChibiOS::I2CBus::clear_all();
#endif

#ifndef HAL_NO_SHARED_DMA
    ChibiOS::Shared_DMA::init();
#endif

    peripheral_power_enable();

    hal.serial(0)->begin(115200);

#ifdef HAL_SPI_CHECK_CLOCK_FREQ
    // optional test of SPI clock frequencies
    ChibiOS::SPIDevice::test_clock_freq();
#endif

    hal.analogin->init();
    hal.scheduler->init();

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_chibios_set_priority(APM_STARTUP_PRIORITY);

    if (stm32_was_watchdog_reset()) {
        // load saved watchdog data
        stm32_watchdog_load((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4);
        utilInstance.last_persistent_data = utilInstance.persistent_data;
    }

    schedulerInstance.hal_initialized();

    g_callbacks->setup();

#if HAL_ENABLE_SAVE_PERSISTENT_PARAMS
    utilInstance.apply_persistent_params();
#endif

#if !defined(DISABLE_WATCHDOG)
#ifdef IOMCU_FW
    stm32_watchdog_init();
#elif !defined(HAL_BOOTLOADER_BUILD)
    // setup watchdog to reset if main loop stops
    if (AP_BoardConfig::watchdog_enabled()) {
        stm32_watchdog_init();
    }

    if (hal.util->was_watchdog_reset()) {
        INTERNAL_ERROR(AP_InternalError::error_t::watchdog_reset);
    }
#endif // IOMCU_FW
#endif // DISABLE_WATCHDOG

    schedulerInstance.watchdog_pat();

    hal.scheduler->set_system_initialized();

    thread_running = true;
    chRegSetThreadName(SKETCHNAME);

    /*
      switch to high priority for main loop
     */
    chThdSetPriority(APM_MAIN_PRIORITY);

    while (true) {
        g_callbacks->loop();

        /*
          give up 50 microseconds of time if the INS loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling
          delay_microseconds_boost() means we have already given up
          time from the main loop, so we don't need to do it again
          here
         */
#if !defined(HAL_DISABLE_LOOP_DELAY) && !APM_BUILD_TYPE(APM_BUILD_Replay)
        if (!schedulerInstance.check_called_boost()) {
            hal.scheduler->delay_microseconds(50);
        }
#endif
        schedulerInstance.watchdog_pat();
    }
    thread_running = false;
}

// 大部分是一些初始化的操作，值得注意的有两个函数
// g_callbacks->setup(); 实际也就是 sub->setup(); 执行了用户层面的初始化
// g_callbacks->loop(); 实际也就是 sub->loop(); 执行了用户层面的主循环
```

##### `setup()`函数

```cpp
// 在 AP_Vehicle.cpp 中
// 注意 sub 类继承自 AP_Vehicle 类

void AP_Vehicle::setup()
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // initialise serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    load_parameters();

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if (AP_BoardConfig::get_sdcard_slowdown() != 0) {
        // user wants the SDcard slower, we need to remount
        sdcard_stop();
        sdcard_retry();
    }
#endif

    // initialise the main loop scheduler
    const AP_Scheduler::Task *tasks;
    uint8_t task_count;
    uint32_t log_bit;
    get_scheduler_tasks(tasks, task_count, log_bit);
    AP::scheduler().init(tasks, task_count, log_bit);

    // time per loop - this gets updated in the main loop() based on
    // actual loop rate
    G_Dt = scheduler.get_loop_period_s();

    // this is here for Plane; its failsafe_check method requires the
    // RC channels to be set as early as possible for maximum
    // survivability.
    set_control_channels();

    // initialise serial manager as early as sensible to get
    // diagnostic output during boot process.  We have to initialise
    // the GCS singleton first as it sets the global mavlink system ID
    // which may get used very early on.
    gcs().init();

    // initialise serial ports
    serial_manager.init();
    gcs().setup_console();

    // Register scheduler_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(scheduler_delay_callback, 5);

#if HAL_MSP_ENABLED
    // call MSP init before init_ardupilot to allow for MSP sensors
    msp.init();
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    // call externalAHRS init before init_ardupilot to allow for external sensors
    externalAHRS.init();
#endif

    // init_ardupilot is where the vehicle does most of its initialisation.
    init_ardupilot();

#if !APM_BUILD_TYPE(APM_BUILD_Replay)
    SRV_Channels::init();
#endif

    // gyro FFT needs to be initialized really late
#if HAL_GYROFFT_ENABLED
    gyro_fft.init(AP::scheduler().get_loop_rate_hz());
#endif
#if HAL_RUNCAM_ENABLED
    runcam.init();
#endif
#if HAL_HOTT_TELEM_ENABLED
    hott_telem.init();
#endif
#if HAL_VISUALODOM_ENABLED
    // init library used for visual position estimation
    visual_odom.init();
#endif

    vtx.init();

#if HAL_SMARTAUDIO_ENABLED
    smartaudio.init();
#endif

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif

    send_watchdog_reset_statustext();

#if HAL_GENERATOR_ENABLED
    generator.init();
#endif

// init EFI monitoring
#if HAL_EFI_ENABLED
    efi.init();
#endif

    gcs().send_text(MAV_SEVERITY_INFO, "ArduPilot Ready");
}
```

---

```cpp
// 上述程序的流程解析
// 1. 加载在 var_info 中的固有参数
// 2. 初始化串口
// 3. 加载任务列表
// 3. 从ROM中加载参数
// 4. 各种初始化操作

// 需要注意的是 
// 在这里加载了用户定义的 scheduler_tasks[]
// 调用init_ardupilot() 函数，做了用户层面大部分的初始化工作
```

##### `loop()`函数

```cpp
// 在 AP_Vehicle.cpp 中
// 注意 sub 类继承自 AP_Vehicle 类

void AP_Vehicle::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();

    if (!done_safety_init) {
        /*
          disable safety if requested. This is delayed till after the
          first loop has run to ensure that all servos have received
          an update for their initial values. Otherwise we may end up
          briefly driving a servo to a position out of the configured
          range which could damage hardware
        */
        done_safety_init = true;
        BoardConfig.init_safety();

        // send RC output mode info if available
        char banner_msg[50];
        if (hal.rcout->get_output_mode_banner(banner_msg, sizeof(banner_msg))) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", banner_msg);
        }
    }
    const uint32_t new_internal_errors = AP::internalerror().errors();
    if(_last_internal_errors != new_internal_errors) {
        AP::logger().Write_Error(LogErrorSubsystem::INTERNAL_ERROR, LogErrorCode::INTERNAL_ERRORS_DETECTED);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Internal Errors %x", (unsigned)new_internal_errors);
        _last_internal_errors = new_internal_errors;
    }
}

// 其实真正要注意的只有第一行
// scheduler.loop() 函数的调用
```

##### `scheduler.loop()`函数

```cpp
// 在 AP_Scheduler.cpp 中

void AP_Scheduler::loop()
{
    // wait for an INS sample
    hal.util->persistent_data.scheduler_task = -3;
    _rsem.give();
    AP::ins().wait_for_sample();
    _rsem.take_blocking();
    hal.util->persistent_data.scheduler_task = -1;

    const uint32_t sample_time_us = AP_HAL::micros();
    
    if (_loop_timer_start_us == 0) {
        _loop_timer_start_us = sample_time_us;
        _last_loop_time_s = get_loop_period_s();
    } else {
        _last_loop_time_s = (sample_time_us - _loop_timer_start_us) * 1.0e-6;
    }

    // Execute the fast loop
    // ---------------------
    if (_fastloop_fn) {
        hal.util->persistent_data.scheduler_task = -2;
        _fastloop_fn();
        hal.util->persistent_data.scheduler_task = -1;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    {
        /*
          for testing low CPU conditions we can add an optional delay in SITL
        */
        auto *sitl = AP::sitl();
        uint32_t loop_delay_us = sitl->loop_delay.get();
        hal.scheduler->delay_microseconds(loop_delay_us);
    }
#endif

    // tell the scheduler one tick has passed
    tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    const uint32_t loop_us = get_loop_period_us();
    uint32_t now = AP_HAL::micros();
    uint32_t time_available = 0;
    const uint32_t loop_tick_us = now - sample_time_us;
    if (loop_tick_us < loop_us) {
        // get remaining time available for this loop
        time_available = loop_us - loop_tick_us;
    }

    // add in extra loop time determined by not achieving scheduler tasks
    time_available += extra_loop_us;
    // update the task info for the fast loop
    perf_info.update_task_info(_num_tasks, loop_tick_us, loop_tick_us > loop_us);

    // run the tasks
    run(time_available);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // move result of AP_HAL::micros() forward:
    hal.scheduler->delay_microseconds(1);
#endif

    if (task_not_achieved > 0) {
        // add some extra time to the budget
        extra_loop_us = MIN(extra_loop_us+100U, 5000U);
        task_not_achieved = 0;
        task_all_achieved = 0;
    } else if (extra_loop_us > 0) {
        task_all_achieved++;
        if (task_all_achieved > 50) {
            // we have gone through 50 loops without a task taking too
            // long. CPU pressure has eased, so drop the extra time we're
            // giving each loop
            task_all_achieved = 0;
            // we are achieving all tasks, slowly lower the extra loop time
            extra_loop_us = MAX(0U, extra_loop_us-50U);
        }
    }

    // check loop time
    perf_info.check_loop_time(sample_time_us - _loop_timer_start_us);
        
    _loop_timer_start_us = sample_time_us;
}
```

---

```cpp
// 对上面程序流程的解释

// 1. 首先等待 IMU 采样数据
// 2. 记录采样花费的时间，计算上一次循环使用的时间
// 3. 执行 fast_loop() 函数
// 4. 通知 scheduler 已经走过一个 tick
// 5. 计算可用时间
// 6. 调用 run() 来执行其他定义在 schedul_task 表中的程序

// 对 3. 如何执行 fast_loop() 的解释

// if (_fastloop_fn) {
//      hal.util->persistent_data.scheduler_task = -2;
//      _fastloop_fn();
//      hal.util->persistent_data.scheduler_task = -1;
//  }
// 在 AP_Vehicle.h 中
// AP_Scheduler scheduler{FUNCTOR_BIND_MEMBER(&AP_Vehicle::fast_loop, void)};
// 初始化中将 _fastloop_fn 与 fast_loop 绑定在一起（函数指针）
```

#### 用户层程序学习

##### `init_ardupilot()`函数

```cpp
// 在 system.cpp 中
// 在该函数中进行用户层的初始化操作

void Sub::init_ardupilot()
{
    BoardConfig.init();
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

#if AC_FENCE == ENABLED
    fence.init();
#endif

    // initialise notify system
    notify.init();

    // initialise battery monitor
    battery.init();

    barometer.init();

#if AP_FEATURE_BOARD_DETECT
    // Detection won't work until after BoardConfig.init()
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 0);
        break;
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        AP_Param::set_by_name("BARO_EXT_BUS", 1);
        break;
    default:
        AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
        break;
    }
#elif CONFIG_HAL_BOARD != HAL_BOARD_LINUX
    AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
#endif
    celsius.init(barometer.external_bus());

    // setup telem slots with serial ports
    gcs().setup_uarts();

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise rc channels including setting mode
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();


    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs
    init_joystick();            // joystick initialization

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
    // This step ncessary so the servo is properly initialized
    camera_mount.set_angle_targets(0, 0, 0);
    // for some reason the call to set_angle_targets changes the mode to mavlink targeting!
    camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // Init baro and determine if we have external (depth) pressure sensor
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate(false);
    barometer.update();

    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        if (barometer.get_type(i) == AP_Baro::BARO_TYPE_WATER) {
            barometer.set_primary_baro(i);
            depth_sensor_idx = i;
            ap.depth_sensor_present = true;
            sensor_health.depth = barometer.healthy(depth_sensor_idx); // initialize health flag
            break; // Go with the first one we find
        }
    }

    if (!ap.depth_sensor_present) {
        // We only have onboard baro
        // No external underwater depth sensor detected
        barometer.set_primary_baro(0);
        ahrs.set_alt_measurement_noise(10.0f);  // Readings won't correspond with rest of INS
    } else {
        ahrs.set_alt_measurement_noise(0.1f);
    }

    leak_detector.init();

    last_pilot_heading = ahrs.yaw_sensor;

    // initialise rangefinder
#if RANGEFINDER_ENABLED == ENABLED
    init_rangefinder();
#endif

    // initialise AP_RPM library
#if RPM_ENABLED == ENABLED
    rpm_sensor.init();
#endif

    // initialise mission library
    mission.init();

    // initialise AP_Logger library
#if LOGGING_ENABLED == ENABLED
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&sub, &Sub::Log_Write_Vehicle_Startup_Messages, void));
#endif

    startup_INS_ground();

#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED

    g2.airspeed.init();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    mainloop_failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // flag that initialisation has completed
    ap.initialised = true;
}
```

##### `fast_loop()`函数

```cpp
// 在 ArduSub.cpp 中
// 在该函数中实现用户层的主循环

void Sub::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    //don't run rate controller in manual or motordetection modes
    if (control_mode != MANUAL && control_mode != MOTOR_DETECT) {
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // send outputs to the motors library
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }

    AP_Vehicle::fast_loop();
}
```

##### `scheduler_tasks`任务列表

```cpp
// 在 ArduSub.cpp 中

const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    SCHED_TASK(fifty_hz_loop,         50,     75,   3),
    SCHED_TASK_CLASS(AP_GPS, &sub.gps, update, 50, 200,   6),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &sub.optflow,             update,         200, 160,   9),
#endif
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    SCHED_TASK(read_rangefinder,      20,    100,  15),
    SCHED_TASK(update_altitude,       10,    100,  18),
    SCHED_TASK(three_hz_loop,          3,     75,  21),
    SCHED_TASK(update_turn_counter,   10,     50,  24),
    SCHED_TASK_CLASS(AP_Baro,             &sub.barometer,    accumulate,          50,  90,  27),
    SCHED_TASK_CLASS(AP_Notify,           &sub.notify,       update,              50,  90,  30),
    SCHED_TASK(one_hz_loop,            1,    100,  33),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180,  36),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550,  39),
#if AC_FENCE == ENABLED
    SCHED_TASK_CLASS(AC_Fence,            &sub.fence,        update,              10, 100,  42),
#endif
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75,  45),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update,              50,  75,  48),
#endif
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  51),
    SCHED_TASK(twentyfive_hz_logging, 25,    110,  54),
    SCHED_TASK_CLASS(AP_Logger,           &sub.logger,       periodic_tasks,     400, 300,  57),
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50,  60),
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75,  63),
#if RPM_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_RPM,              &sub.rpm_sensor,   update,              10, 200,  66),
#endif
    SCHED_TASK_CLASS(Compass,             &sub.compass,      cal_update,         100, 100,  69),
    SCHED_TASK(terrain_update,        10,    100,  72),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &sub.g2.gripper,   update,              10,  75,  75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75,  78),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75,  81),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75,  84),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75,  87),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75,  90),
#endif
    SCHED_TASK(read_airspeed,         10,    100,  93),
};

// 注意 任务列表(scheduler_tasks) 中可以通过两种方式来定义 进程任务
// 1. SCHED_TASK()
// 2. SCHED_TASK_CLASS()
```

---

```CPP
// 1. SCHED_TASK() 定义在 ArsuSub.cpp 中 默认是 sub 类
#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros, priority)

// 2. SCHED_TASK_CLASS() 定义在 AP_Scheduler.h 中
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros, _priority) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(classname, func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros,        \
    .priority = _priority \
}
```

---

```cpp
// 通过下面的函数来实现对 自定义的 任务列表 的加载
// 实际在 setup() 中被调用
void Sub::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}
```

#### 电机控制程序学习

##### 输入通道

```cpp
// 在 Sub.h 中

// primary input control channels
RC_Channel *channel_roll;
RC_Channel *channel_pitch;
RC_Channel *channel_throttle;
RC_Channel *channel_yaw;
RC_Channel *channel_forward;
RC_Channel *channel_lateral;
```

定义了六个输入通道，每一个通道分别与水下机器人的一个方向上的运动（三个旋转，三个平移）对应，一个通道上的输入量会影响到水下机器人的多个推进器。

|      输入通道      | 对应运动 |
| :----------------: | :------: |
|   `channel_roll`   |   横滚   |
|  `channel_pitch`   |   俯仰   |
|   `channel_yaw`    |   偏航   |
| `channel_throttle` |   上下   |
| `channel_forward`  |   前后   |
| `channel_lateral`  |   左右   |

##### 输出通道

在`pixhawk`控制板上定义有8个主输出通道和6个辅助输出通道。其中8个主输出通道用作水下机器人推进器的控制输出，而其他6个辅助输出通道则可以作为相机云台，灯光亮度的控制输出。

##### 电机控制流程

###### 电机类型

在 `sub` 中使用的电机类型是 `AP_Motors6DOF`，此类型电机是专门为 `ROV`的电机控制而实现的。可以方便地实现 `ROV`的六自由度的运动控制。

```cpp
// 在 sub.h sub 类 中对电机进行了定义
AP_Motors6DOF motors;

// 电机类的继承关系如下：
//     AP_Motors
//     	  AP_MotorsMulticopter
//     		 AP_MotorsMatrix
//     			AP_Motors6DOF
```

###### 流程梳理

1. 初始化

```cpp
// 在 init_ardupilot() 中调用 init_rc_in()
// 对各个输入通道(主要看前6个)进行了初始化操作
// 包括输入的范围限制 以及 死区设置

void Sub::init_rc_in()
{
    channel_pitch    = RC_Channels::rc_channel(0);
    channel_roll     = RC_Channels::rc_channel(1);
    channel_throttle = RC_Channels::rc_channel(2);
    channel_yaw      = RC_Channels::rc_channel(3);
    channel_forward  = RC_Channels::rc_channel(4);
    channel_lateral  = RC_Channels::rc_channel(5);

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_throttle->set_range(1000);
    channel_forward->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_lateral->set_angle(ROLL_PITCH_INPUT_MAX);

    // set default dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(40);
    channel_forward->set_default_dead_zone(30);
    channel_lateral->set_default_dead_zone(30);

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // initialize rc input to 1500 on control channels (rather than 0)
    for (int i = 0; i < 6; i++) {
        RC_Channels::set_override(i, 1500);
    }

    RC_Channels::set_override(6, 1500); // camera pan channel
    RC_Channels::set_override(7, 1500); // camera tilt channel

    RC_Channel* chan = RC_Channels::rc_channel(8);
    uint16_t min = chan->get_radio_min();
    RC_Channels::set_override(8, min); // lights 1 channel

    chan = RC_Channels::rc_channel(9);
    min = chan->get_radio_min();
    RC_Channels::set_override(9, min); // lights 2 channel

    RC_Channels::set_override(10, 1100); // video switch
#endif
}
```

```cpp
// 在 init_ardupilot() 中调用 init_rc_out()
// 对水下机器人的推进器结构进行了初始化
// 默认是 SUB_FRAME_VECTORED 在 parameter.cpp 中定义

void Sub::init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_loop_rate(scheduler.get_loop_rate_hz());
    motors.init((AP_Motors::motor_frame_class)g.frame_configuration.get(), AP_Motors::motor_frame_type::MOTOR_FRAME_TYPE_PLUS);
    motors.convert_pwm_min_max_param(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    motors.update_throttle_range();

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();
}
```

2. 输出控制

```cpp
// 在 fast_loop() 中 的以下程序 输出电机控制

// send outputs to the motors library
motors_output();
```

###### 自定义推进器结构

```cpp
// 1. 在 AP_Motors6DOF.h 中 sub_frame_t 中添加自定义的 推进器结构名 
// Supported frame types
typedef enum {
    SUB_FRAME_BLUEROV1,
    SUB_FRAME_VECTORED,
    SUB_FRAME_VECTORED_6DOF,
    SUB_FRAME_VECTORED_6DOF_90DEG,
    SUB_FRAME_SIMPLEROV_3,
    SUB_FRAME_SIMPLEROV_4,
    SUB_FRAME_SIMPLEROV_5,
    SUB_FRAME_CUSTOM
} sub_frame_t;

// 2. 在 AP_Motors6DOF.cpp 中 setup_motors() 中添加对应的 case 并 调用 add_motor_raw_6dof() 添加电机

// 3. 在 parameters.cpp 中 修改使用的推进器结构
```

#### 水下机器人控制模式

`sub`中默认的控制模式是 `MANNUAL`模式

```cpp
// 在 sub.cpp 中
// sub 类初始化
control_mode(MANUAL),
```

在源码中一共定义了10种控制模式

```cpp
// 在 define.h 中
enum control_mode_t : uint8_t {
    STABILIZE =     0,  // manual angle with manual depth/throttle
    ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
    ALT_HOLD =      2,  // manual angle with automatic depth/throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    MANUAL =       19,  // Pass-through input with no stabilization
    MOTOR_DETECT = 20   // Automatically detect motors orientation
};
```

##### 模式控制流程

###### 模式初始化

在程序启动，初始化阶段，控制模式初始化为默认的`MANUAL`模式

###### 模式更新

官方源码中，通过手柄的按键来选择水下机器人的控制模式。

函数的调用流程如下所示

```cpp
// 1. scheduler_tasks[]中注册了GCS 的消息接收更新线程
SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180,  36)
    
// 2. update_receive()函数对接收的数据进行mavlink包判定
// 如果是完整的包则调用 packetReceived() 
if (mavlink_parse_char(chan, c, &msg, &status)) {
            hal.util->persistent_data.last_mavlink_msgid = msg.msgid;
            packetReceived(status, msg);
            parsed_packet = true;
            gcs_alternative_active[chan] = false;
            alternative.last_mavlink_ms = now_ms;
            hal.util->persistent_data.last_mavlink_msgid = 0;
        }

// 3. packetReceived()中调用了 handleMessage()函数
handleMessage(msg);

// 4. handleMessage()中对包进行判断，如果是手柄数据则进行解包并调用
// transform_manual_control_to_rc_override()函数

case MAVLINK_MSG_ID_MANUAL_CONTROL: {     // MAV ID: 69
    if (msg.sysid != sub.g.sysid_my_gcs) {
        break;    // Only accept control from our gcs
    }
    mavlink_manual_control_t packet;
    mavlink_msg_manual_control_decode(&msg, &packet);

    if (packet.target != sub.g.sysid_this_mav) {
        break; // only accept control aimed at us
    }

   		sub.transform_manual_control_to_rc_override(packet.x,packet.y,packet.z,packet.r,packet.buttons);

    sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
    // a RC override message is considered to be a 'heartbeat'
    // from the ground station for failsafe purposes
    gcs().sysid_myggcs_seen(AP_HAL::millis());
    break;
}


// 5. transform_manual_control_to_rc_override()对按键信息进行处理

// Act if button is pressed
    // Only act upon pressing button and ignore holding. This provides compatibility with Taranis as joystick.
for (uint8_t i = 0 ; i < 16 ; i++) {
    if ((buttons & (1 << i))) {
        handle_jsbutton_press(i,shift,(buttons_prev & (1 << i)));
        // buttonDebounce = tnow_ms;
    } else if (buttons_prev & (1 << i)) {
        handle_jsbutton_release(i, shift);
    }
}

// 6. handle_jsbutton_press()中有对控制模式进行设置

// handle_jsbutton_press()部分代码

case JSButton::button_function_t::k_mode_manual:
    set_mode(MANUAL, ModeReason::RC_COMMAND);
    break;
case JSButton::button_function_t::k_mode_stabilize:
    set_mode(STABILIZE, ModeReason::RC_COMMAND);
    break;
case JSButton::button_function_t::k_mode_depth_hold:
    set_mode(ALT_HOLD, ModeReason::RC_COMMAND);
    break;
case JSButton::button_function_t::k_mode_auto:
    set_mode(AUTO, ModeReason::RC_COMMAND);
    break;
case JSButton::button_function_t::k_mode_guided:
    set_mode(GUIDED, ModeReason::RC_COMMAND);
    break;
case JSButton::button_function_t::k_mode_circle:
    set_mode(CIRCLE, ModeReason::RC_COMMAND);
    break;
case JSButton::button_function_t::k_mode_acro:
    set_mode(ACRO, ModeReason::RC_COMMAND);
    break;
case JSButton::button_function_t::k_mode_poshold:
    set_mode(POSHOLD, ModeReason::RC_COMMAND);
    break;

// 7. set_mode() 对对应模式进行初始化工作，并修改当前模式

// 8. 在 fast_loop() 中调用 update_flight_mode() 来进行相应模式的控制
```

###### 模式运行

在 fast_loop() 中调用 update_flight_mode() 来进行相应模式的控制

##### 定义新的控制模式

###### 添加控制模式名

首先注意在`define.h`中对水下机器人的控制模式进行了枚举

```cpp
// Auto Pilot Modes enumeration
enum control_mode_t : uint8_t {
    STABILIZE =     0,  // manual angle with manual depth/throttle
    ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
    ALT_HOLD =      2,  // manual angle with automatic depth/throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    MANUAL =       19,  // Pass-through input with no stabilization
    MOTOR_DETECT = 20   // Automatically detect motors orientation
};
```

定义新的控制模式时首先在枚举类型中添加上控制模式名称

###### 添加相关函数声明

在 `Sub.h` 文件中添加控制模式的两个主体函数，一个是初始化函数，另一个是运行函数。

```cpp
bool newmode_init(void);

void newmode_run();
```

###### 编写控制函数

新建一个文件`control_newmode.cpp`

```cpp
// 具体内容如下：
#include "sub.h"

bool Sub::newmode_init()
{
	/* your code */
}

void Sub::newmode_run()
{
	/* your code */
}
```

###### 上层调用函数修改

根据上述模式控制流程的分析，需要修改的上层调用函数包括如下：

```cpp
1. set_mode()函数
    增加对应模式的判断和初始化函数调用
2. update_flight_mode()函数
    增加对应模式判断和运行函数的调用
```

###### 添加对应按键控制

在 `AP_JSButton.h`中，有对按键的功能进行枚举，需要添加对应的按键来触发新增的控制模式。

```cpp
// 部分枚举

k_mode_manual           = 5,            ///< enter enter manual mode
k_mode_stabilize        = 6,            ///< enter stabilize mode
k_mode_depth_hold       = 7,            ///< enter depth hold mode
k_mode_poshold          = 8,            ///< enter poshold mode
k_mode_auto             = 9,            ///< enter auto mode
k_mode_circle           = 10,           ///< enter circle mode
k_mode_guided           = 11,           ///< enter guided mode
k_mode_acro             = 12,           ///< enter acro mode
```

除此之外，需要在 `joystick.cpp` 中的 `handle_jsbutton_press()` 中增加新添加的按键功能判断，并调用 `set_mode()` 来设置相应的模式。

#### 姿态控制方法

姿态控制即控制`roll`，`pitch`，`yaw`三个姿态角，每个姿态分别使用的是一个两级的串级`PID`控制器，外环是`P`控制器，内环是`PID`控制器。外环输入期望姿态角，输出期望角速度，作为内环的输入。内环输出的是电机控制量。

外环的实现在不同的控制模式的`mode_run()`函数中，而内环的实现是在姿态控制函数中。

值得注意的是，在内环使用`PID`控制时应用了抗积分饱和的策略，即达到饱和时只有反向的误差才叠加，否则不叠加积分项。

##### 控制流程

```bash
fast_loop()  # 主循环
	|--ins.update() -> wait_for_sample()  # 姿态传感器采样
	|--attitude_control.rate_controller_run()  # 运行姿态控制
	|					|--update_throttle_rpy_mix()
	|					|--get_gyro_latest()
	|					|--set_roll()  # 使用PID计算控制量
	|					|		|--get_rate_roll_pid().update_all()
    |					|--set_pitch()  # 使用PID计算控制量
	|					|		|--get_rate_pitch_pid().update_all()
    |					|--set_yaw()  # 使用PID计算控制量
	|					|		|--get_rate_yaw_pid().update_all()
	|					|--control_monitor_update()
	|--motor_output()  # 输出控制给电机
	|		|--output_armed_stabilizing()  # 将各通道油门转化为各电机油门
	|		|--output_rpyt()  # 将油门值转化为PWM值
	...
```

#### `Mavlink`消息通信

##### 初始化

在`setup()`中，在运行`init_ardupilot()`前有如下几个函数是与`Mavlink`消息相关的

```c++
// initialise serial manager as early as sensible to get
// diagnostic output during boot process.  We have to initialise
// the GCS singleton first as it sets the global mavlink system ID
// which may get used very early on.
gcs().init();

// initialise serial ports
serial_manager.init();
gcs().setup_console();

// Register scheduler_delay_cb, which will run anytime you have
// more than 5ms remaining in your call to hal.scheduler->delay
hal.scheduler->register_delay_callback(scheduler_delay_callback, 5);
```

根据注释可知，`gcs().init()`是设置了全局参数`system ID`其值为1

```c++
// 在 CS.cpp 中 gcs().init() 函数定义如下
void GCS::init()
{
    mavlink_system.sysid = sysid_this_mav();
}

// 在 CS_Sub.cpp ，sysid_this_mav()定义如下
uint8_t GCS_Sub::sysid_this_mav() const
{
    return sub.g.sysid_this_mav;
}

// 在 arameter.cpp ，设置了sysid_this_mav的值
GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID)；
    
// config.h中，定义了MAV_SYSTEM_ID的值
#ifndef MAV_SYSTEM_ID
# define MAV_SYSTEM_ID          1
#endif
```

在这之后，建立通信的通道

```c++
serial_manager.init(); // 初始化了串口
gcs().setup_console(); // 找到一个可用的串口来作为消息收发的通道
```

##### 消息接收

消息的接收是一个独立的任务，在任务列表中有注册

```c++
SCHED_TASK_CLASS(GCS,(GCS*)&sub._gcs,update_receive,400,180,36)；
```

进入`update_receive`

```c++
// 循环接收来自每个通道的信息
void GCS::update_receive(void)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        chan(i)->update_receive();
    }
    // also update UART pass-thru, if enabled
    update_passthru();
}

// chan(i)->update_receive()实际执行下面的函数
GCS_MAVLINK::update_receive(uint32_t max_time_us)；

// 重要的是下面这段，调用函数来解包
// Try to get a new message
if (mavlink_parse_char(chan, c, &msg, &status)) {
    hal.util->persistent_data.last_mavlink_msgid = msg.msgid;
    packetReceived(status, msg);
    parsed_packet = true;
    gcs_alternative_active[chan] = false;
    alternative.last_mavlink_ms = now_ms;
    hal.util->persistent_data.last_mavlink_msgid = 0;
}

// 如果是一个mavlink消息包，则会执行
packetReceived(status, msg);

// 在这个函数最后调用了消息处理函数
handleMessage(msg);

// 根据msg.msgid来分别处理不同的消息，一些通用消息处理转入下面函数处理
handle_common_message(msg);
```

##### 消息发送

消息的发送也是一个独立的任务，在任务列表中有注册，就在消息接收下面一行

```c++
SCHED_TASK_CLASS(GCS, (GCS*)&sub._gcs,   update_send,  400, 550,  39),
```

进入`update_send`

```c++
// 在一个循环内 分不同消息类型来发送消息
while (AP_HAL::millis() - start < 5) { // spend a max of 5ms sending messages.
    ...};

// 调用函数 do_try_send_message()
// do_try_send_message() 中有调用如下
if (!try_send_message(id)) {};

// try_send_message(id) 根据不同的 msg_id 来发送消息
switch(id) {

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude();
        break;
        
        ...};

// 如 MSG_ATTITUDE 则调用send_attitude()来发送姿态消息
void GCS_MAVLINK::send_attitude() const
{
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}    
```

##### 自定义消息使用

在`modules`文件夹下，有`mavlink`子文件夹，在该文件夹下有`message_definitions/v1.0`文件夹。修改`common.xml`文件，即可实现自定义消息。需要注意的是自定义消息的`id`不能与其他消息产生冲突

```xml
<!-- 自定义消息参考原有消息定义即可 -->
<message id="66" name="REQUEST_DATA_STREAM">
      <description>Request a data stream.</description>
      <field type="uint8_t" name="target_system">The target requested to send the message stream.</field>
      <field type="uint8_t" name="target_component">The target requested to send the message stream.</field>
      <field type="uint8_t" name="req_stream_id">The ID of the requested data stream</field>
      <field type="uint16_t" name="req_message_rate" units="Hz">The requested message rate</field>
      <field type="uint8_t" name="start_stop">1 to start sending, 0 to stop sending.</field>
    </message>
```

