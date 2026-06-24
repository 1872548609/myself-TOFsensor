/*
 * ============================================================================
 * 逐句中文注释版说明
 * ----------------------------------------------------------------------------
 * 本文件以用户提供的 main.c 为原始文本生成。
 * 所有原有 C 代码行均按原顺序保留，未改动任意原始代码字符；
 * 本版仅额外插入中文注释，便于逐行分析状态机、按键、标定、输出与超时流程。
 * ============================================================================
 */

/**
 * @file main.c
 * @brief YC02 TOF01 工业传感器主程序。
 *
 * 功能：
 * 1. 按键1（PA7）二段标定，取两次距离中点为设定值。
 * 2. 大于设定值+应差，连续 N 帧 -> 触发；小于设定值-应差，连续 M 帧 -> 遮光。
 * 3. 按键2（PA8）切换 NO / NC 逻辑。
 * 4. 打开测试宏后，USART2/RS485 输出 ON、OFF、以及未确认候选的确认时间。
 */
/* 定义 main.c 的编译单元标识，供 main.h 内可能的条件编译使用。 */
#define _MAIN_C_

/* 引入板级初始化、GPIO、LED、按键、SysTick 等平台接口声明。 */
#include "platform.h"
/* 引入 TOF 串口接收、帧解析、USART2 调试输出等接口声明。 */
#include "usart_interrupt.h"
/* 引入本文件对外需要的主程序声明。 */
#include "main.h"

/* ============================ 用户可调参数区 ============================ */
/* 设置距离迟滞宽度为 20 mm；触发阈值为设定值+20，复位阈值为设定值-20。 */
#define SENSOR_HYSTERESIS_MM                   20U
/* 设置由遮光进入触发状态所需的连续有效 TOF 帧数为 3 帧。 */
#define SENSOR_ON_CONFIRM_FRAMES               3U
/* 设置由触发返回遮光状态所需的连续有效 TOF 帧数为 3 帧。 */
#define SENSOR_OFF_CONFIRM_FRAMES              3U

/* 设定 TOF 帧最低可信度；低于 70 的帧不参与标定和输出判定。 */
#define SENSOR_CONFIDENCE_MIN                  70U
/* 设定允许参与判定的最小距离，避免 0 或异常近距离被当作有效测量。 */
#define SENSOR_DISTANCE_MIN_VALID_MM           1
/* 设定允许参与判定的最大距离，超出该值的帧视为无效。 */
#define SENSOR_DISTANCE_MAX_VALID_MM           30000

/* 设定“未出现完整有效协议帧”的通信超时时间为 150 ms。 */
#define SENSOR_FRAME_TIMEOUT_MS                150U
/* 设定“未出现可信且距离合法测量”的业务超时时间为 200 ms。 */
#define SENSOR_MEASUREMENT_TIMEOUT_MS          200U

/* 每一段标定均取 7 帧中位数。必须为奇数，且不大于 9。 */
/* 每一次标定收集 7 个有效距离点，随后用中位数代表该标定点。 */
#define SENSOR_CAL_SAMPLE_COUNT                7U
/* 第一段或第二段采样最多允许持续 2000 ms，超时即标定失败。 */
#define SENSOR_CAL_CAPTURE_TIMEOUT_MS          2000U
/* 第一段完成后，最多等待 30 秒按下第二次标定按键。 */
#define SENSOR_CAL_WAIT_SECOND_TIMEOUT_MS      30000U
/* 两标定点除去双边迟滞外还必须额外拉开 20 mm，防止标定点过近。 */
#define SENSOR_CAL_EXTRA_GUARD_MM              20U

/* 按键输入状态需稳定 30 ms 才认为是真实状态变化。 */
#define SENSOR_KEY_DEBOUNCE_MS                 30U

/* 测试宏：1=经 USART2/RS485 输出切换时间；0=不产生测试日志。 */
/* 开启时通过 USART2/RS485 输出状态、标定、响应时间调试日志。 */
#define SENSOR_TIMING_TEST_ENABLE              1U

/* 1=候选帧未达到确认数就退出时，也输出 ON_ABORT/OFF_ABORT。 */
/* 开启时，候选帧未满足确认次数就中断也会打印 ABORT 日志。 */
#define SENSOR_TIMING_LOG_ABORTED_CANDIDATE    1U

/*
 * 故障或未标定时 Q1 的状态。
 * 0：Q1 截止，OUT 开路；1：Q1 导通，OUT 被拉低。
 * 当前默认“失效开路”。若现场 PLC 需要相反的故障电平，仅改这两个宏。
 */
/* TOF 故障时令 Q1 截止，使 NPN 开集电极 OUT 保持开路。 */
#define SENSOR_FAULT_TRANSISTOR_ON             0U
/* 未标定时令 Q1 截止，使 OUT 保持开路，避免误动作。 */
#define SENSOR_UNCAL_TRANSISTOR_ON             0U

/* 上电的默认输出模式：0=NO；1=NC。 */
/* 上电默认采用 NO 逻辑：只有进入触发状态时 Q1 才导通。 */
#define SENSOR_DEFAULT_OUTPUT_MODE_NC          0U
/* ====================================================================== */

/* 开始定义枚举类型，用名称表达有限状态集合。 */
typedef enum
{
    /* 状态值 0：尚未完成两段标定，不能进行正常距离输出。 */
    SENSOR_STATE_UNCALIBRATED = 0,
    /* 状态值 1：当前为遮光/未触发稳定状态。 */
    SENSOR_STATE_BLOCKED,
    /* 状态值 2：当前为有效触发稳定状态。 */
    SENSOR_STATE_TRIGGERED,
    /* 状态值 3：TOF 通信或可信测量超时，进入故障安全状态。 */
    SENSOR_STATE_FAULT
/* 结束当前类型定义，并给该类型命名。 */
} SensorState_t;

/* 开始定义枚举类型，用名称表达有限状态集合。 */
typedef enum
{
    /* NO 逻辑：检测触发时才使 Q1 导通。 */
    SENSOR_OUTPUT_NO = 0,
    /* NC 逻辑：检测未触发/遮光时使 Q1 导通。 */
    SENSOR_OUTPUT_NC
/* 结束当前类型定义，并给该类型命名。 */
} SensorOutputMode_t;

/* 开始定义枚举类型，用名称表达有限状态集合。 */
typedef enum
{
    /* 标定状态机空闲：等待第一次按键标定。 */
    CAL_STEP_IDLE = 0,
    /* 正在采集第一标定点的 7 帧有效距离。 */
    CAL_STEP_CAPTURE_1,
    /* 第一标定点已经完成，等待第二次按键。 */
    CAL_STEP_WAIT_2,
    /* 正在采集第二标定点的 7 帧有效距离。 */
    CAL_STEP_CAPTURE_2
/* 结束当前类型定义，并给该类型命名。 */
} CalibrationStep_t;

/* 开始定义枚举类型，用名称表达有限状态集合。 */
typedef enum
{
    /* 无临时指示，LED 按正常状态显示。 */
    INDICATION_NONE = 0,
    /* 临时显示标定成功指示。 */
    INDICATION_CAL_SUCCESS,
    /* 临时显示标定失败指示。 */
    INDICATION_CAL_FAIL,
    /* 临时显示 NO/NC 模式切换指示。 */
    INDICATION_MODE_CHANGED
/* 结束当前类型定义，并给该类型命名。 */
} Indication_t;

/* 开始定义结构体类型，把相关运行数据集中保存。 */
typedef struct
{
    /* 保存本次轮询读到的原始按键电平。 */
    uint8_t raw;
    /* 保存经过消抖确认后的稳定按键电平。 */
    uint8_t stable;
    /* 记录原始电平最近一次变化的毫秒时间戳。 */
    uint32_t raw_change_tick;
/* 结束当前类型定义，并给该类型命名。 */
} KeyDebounce_t;

/* 开始定义结构体类型，把相关运行数据集中保存。 */
typedef struct
{
    /* 保存传感器业务状态机的当前稳定状态。 */
    SensorState_t state;
    /* 保存当前 NO/NC 输出逻辑模式。 */
    SensorOutputMode_t output_mode;
    /* 保存二段标定状态机所处阶段。 */
    CalibrationStep_t cal_step;

    /* 标记是否已经得到可用的两段标定结果。 */
    uint8_t calibrated;
    /* 保存第一标定点的 7 帧距离中位数。 */
    int16_t cal_point_1_mm;
    /* 保存第二标定点的 7 帧距离中位数。 */
    int16_t cal_point_2_mm;
    /* 保存最终触发基准：两标定点的算术中点。 */
    int16_t setpoint_mm;

    /* 记录当前连续满足“高于上阈值”的触发候选帧数。 */
    uint8_t on_count;
    /* 记录当前连续满足“低于下阈值”的遮光候选帧数。 */
    uint8_t off_count;
    /* 记录第一次进入 ON 候选区的毫秒时间，用于统计确认耗时。 */
    uint32_t on_start_tick;
    /* 记录第一次进入 OFF 候选区的毫秒时间，用于统计确认耗时。 */
    uint32_t off_start_tick;

    /* 保存最近接收到的 TOF 距离，用于调试及 ABORT 日志。 */
    int16_t last_distance_mm;
    /* 保存最近接收到的 TOF 置信度，用于状态观察。 */
    uint8_t last_confidence;
    /* 记录最近一帧“置信度和距离均合法”测量的时间戳。 */
    uint32_t last_good_measurement_tick;

    /* 保存一段标定中的多帧距离样本，供中位数滤波使用。 */
    int16_t cal_samples[SENSOR_CAL_SAMPLE_COUNT];
    /* 记录当前已经收集到的标定样本数。 */
    uint8_t cal_sample_count;
    /* 记录当前标定阶段开始时间，用于标定超时检测。 */
    uint32_t cal_step_start_tick;

    /* 保存当前需要优先显示的临时指示类型。 */
    Indication_t indication;
    /* 保存临时 LED 指示结束的绝对时间点。 */
    uint32_t indication_until_tick;
/* 结束当前类型定义，并给该类型命名。 */
} SensorApp_t;

/* 定义全局传感器运行上下文并清零初始化；仅本文件可访问。 */
static SensorApp_t g_sensor = {0};
/* 定义按键 1 的独立消抖上下文并清零初始化。 */
static KeyDebounce_t g_key1 = {0};
/* 定义按键 2 的独立消抖上下文并清零初始化。 */
static KeyDebounce_t g_key2 = {0};

/* 定义时间间隔比较函数；使用无符号减法可自然兼容 tick 回绕。 */
static uint8_t Sensor_TimeElapsed(uint32_t now, uint32_t start, uint32_t interval)
{
    /* 以 now-start 计算经过时间，到达 interval 时返回 1，否则返回 0。 */
    return ((uint32_t)(now - start) >= interval) ? 1U : 0U;
}

/* 用于“到期时间点”判断，兼容 uint32_t 时间戳自然回绕。 */
/* 定义绝对到期点判断函数；将差值转为有符号数以兼容 uint32_t 回绕。 */
static uint8_t Sensor_TimeReached(uint32_t now, uint32_t deadline)
{
    /* 当 now 已经到达或超过 deadline 时返回 1。 */
    return (((int32_t)(now - deadline)) >= 0) ? 1U : 0U;
}

/* 定义两个有符号距离值的无符号绝对差函数。 */
static uint16_t Sensor_AbsDiffU16(int16_t a, int16_t b)
{
    /* 使用 32 位临时变量，避免 int16_t 相减在边界值附近溢出。 */
    int32_t diff;

    /* 先把两个 int16_t 扩展为 int32_t 后再相减。 */
    diff = (int32_t)a - (int32_t)b;

    /* 若差值为负数，说明 a 小于 b，需要取相反数。 */
    if (diff < 0)
    {
        /* 将负差值转为正值，得到绝对距离差。 */
        diff = -diff;
    }

    /* 把已保证非负的差值转换为 uint16_t 返回。 */
    return (uint16_t)diff;
}

/* 定义单帧测量合法性过滤函数；所有标定和距离判定均依赖它。 */
static uint8_t Sensor_IsMeasurementValid(const TOF_Frame_t *frame)
{
    /* 先防御性检查指针，空帧绝不允许继续解引用。 */
    if (frame == 0)
    {
        /* 空指针表示无有效帧，直接返回无效。 */
        return 0U;
    }

    /* 检查模块给出的 confidence 是否达到设定下限。 */
    if (frame->confidence < SENSOR_CONFIDENCE_MIN)
    {
        /* 低置信度帧不作为遮光，也不参与连续确认。 */
        return 0U;
    }

    /* 检查距离是否落在允许的业务范围内。 */
    if ((frame->distance_mm < SENSOR_DISTANCE_MIN_VALID_MM) ||
        (frame->distance_mm > SENSOR_DISTANCE_MAX_VALID_MM))
    {
        /* 越界距离视为测量无效。 */
        return 0U;
    }

    /* 所有过滤条件均通过，返回有效。 */
    return 1U;
}

/* 定义调试事件统一前缀输出函数。 */
static void Sensor_DebugPrefix(const char *event)
{
/* 仅测试宏打开时才编译调试发送代码。 */
#if SENSOR_TIMING_TEST_ENABLE
    /* 先输出事件字段名 EV=。 */
    USART2_DebugWriteString("EV=");
    /* 输出具体事件字符串，如 ON、OFF、CAL_OK。 */
    USART2_DebugWriteString(event);
/* 切换到条件编译的另一分支，用于关闭功能时保持编译告警为零。 */
#else
    /* 测试关闭时显式引用参数以抑制未使用参数警告。 */
    (void)event;
/* 结束本组条件编译控制。 */
#endif
}

/* 定义一次 ON/OFF/ABORT 事件的标准化调试输出函数。 */
static void Sensor_DebugTimingEvent(const char *event,
                                    /* 声明局部变量，为当前函数的判断、计算或状态更新保存临时数据。 */
                                    int16_t distance_mm,
                                    /* 声明局部变量，为当前函数的判断、计算或状态更新保存临时数据。 */
                                    uint8_t count,
                                    /* 声明局部变量，为当前函数的判断、计算或状态更新保存临时数据。 */
                                    uint32_t duration_ms)
{
/* 测试模式下才产生串口调试流量，避免影响正式运行实时性。 */
#if SENSOR_TIMING_TEST_ENABLE
    /* 输出 EV=<事件名称>。 */
    Sensor_DebugPrefix(event);
    /* 输出距离字段 D=。 */
    USART2_DebugWriteString(",D=");
    /* 输出本次确认/中止时对应的最近距离。 */
    USART2_DebugWriteS16(distance_mm);
    /* 输出确认帧数字段 N=。 */
    USART2_DebugWriteString(",N=");
    /* 输出本次累计的候选有效帧数。 */
    USART2_DebugWriteU16((uint16_t)count);
    /* 输出耗时字段 T=。 */
    USART2_DebugWriteString(",T=");
    /* 输出从第一候选帧到状态确认的毫秒数。 */
    USART2_DebugWriteU32(duration_ms);
    /* 以 ms 和换行结束一条完整日志。 */
    USART2_DebugWriteString("ms\r\n");
/* 切换到条件编译的另一分支，用于关闭功能时保持编译告警为零。 */
#else
    /* 执行当前语句。 */
    (void)event;
    /* 执行当前语句。 */
    (void)distance_mm;
    /* 执行当前语句。 */
    (void)count;
    /* 执行当前语句。 */
    (void)duration_ms;
/* 结束本组条件编译控制。 */
#endif
}

/* 定义标定结果的调试打印函数。 */
static void Sensor_DebugCalibration(const char *event)
{
/* 仅测试模式下输出标定点、中点和迟滞参数。 */
#if SENSOR_TIMING_TEST_ENABLE
    /* 输出 EV=<标定事件> 前缀。 */
    Sensor_DebugPrefix(event);
    /* 执行当前变量或状态字段的赋值/更新操作。 */
    USART2_DebugWriteString(",P1=");
    /* 调用下层函数完成当前步骤的具体动作。 */
    USART2_DebugWriteS16(g_sensor.cal_point_1_mm);
    /* 执行当前变量或状态字段的赋值/更新操作。 */
    USART2_DebugWriteString(",P2=");
    /* 调用下层函数完成当前步骤的具体动作。 */
    USART2_DebugWriteS16(g_sensor.cal_point_2_mm);
    /* 执行当前变量或状态字段的赋值/更新操作。 */
    USART2_DebugWriteString(",SP=");
    /* 调用下层函数完成当前步骤的具体动作。 */
    USART2_DebugWriteS16(g_sensor.setpoint_mm);
    /* 执行当前变量或状态字段的赋值/更新操作。 */
    USART2_DebugWriteString(",HYS=");
    /* 调用下层函数完成当前步骤的具体动作。 */
    USART2_DebugWriteU16(SENSOR_HYSTERESIS_MM);
    /* 调用下层函数完成当前步骤的具体动作。 */
    USART2_DebugWriteString("\r\n");
/* 切换到条件编译的另一分支，用于关闭功能时保持编译告警为零。 */
#else
    /* 执行当前语句。 */
    (void)event;
/* 结束本组条件编译控制。 */
#endif
}

/* 定义 NO/NC 输出模式切换事件的日志函数。 */
static void Sensor_DebugMode(void)
{
/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_TEST_ENABLE
    /* 输出模式切换事件名。 */
    USART2_DebugWriteString("EV=MODE,");
    /* 根据当前枚举值输出 NO 或 NC 文本。 */
    USART2_DebugWriteString((g_sensor.output_mode == SENSOR_OUTPUT_NO) ? "NO" : "NC");
    /* 结束模式日志。 */
    USART2_DebugWriteString("\r\n");
/* 结束本组条件编译控制。 */
#endif
}

/* 定义唯一的物理输出映射函数：所有业务状态最终都通过这里驱动 Q1。 */
static void Sensor_ApplyOutput(void)
{
    /* 声明局部变量，表示是否应使 NPN 输出晶体管 Q1 导通。 */
    uint8_t transistor_on;

    /* 先默认 Q1 截止，后续根据状态再覆盖。 */
    transistor_on = 0U;

    /* 按当前业务状态选择 Q1 的最终导通/截止逻辑。 */
    switch (g_sensor.state)
    {
        /* 稳定触发状态分支。 */
        case SENSOR_STATE_TRIGGERED:
            /* NO 时触发导通，NC 时触发截止。 */
            transistor_on = (g_sensor.output_mode == SENSOR_OUTPUT_NO) ? 1U : 0U;
            /* 结束触发状态分支。 */
            break;

        /* 稳定遮光状态分支。 */
        case SENSOR_STATE_BLOCKED:
            /* NC 时遮光导通，NO 时遮光截止。 */
            transistor_on = (g_sensor.output_mode == SENSOR_OUTPUT_NC) ? 1U : 0U;
            /* 结束遮光状态分支。 */
            break;

        /* TOF 故障状态分支。 */
        case SENSOR_STATE_FAULT:
            /* 使用独立故障宏决定 Q1 状态。 */
            transistor_on = SENSOR_FAULT_TRANSISTOR_ON;
            /* 结束故障状态分支。 */
            break;

        /* 未标定状态分支。 */
        case SENSOR_STATE_UNCALIBRATED:
        /* 处理未覆盖状态的默认安全逻辑。 */
        default:
            /* 使用独立未标定宏决定 Q1 状态。 */
            transistor_on = SENSOR_UNCAL_TRANSISTOR_ON;
            /* 结束未标定状态分支。 */
            break;
    }

    /* 将逻辑 transistor_on 转换为平台 ENABLE/DISABLE 并实际写入输出 GPIO。 */
    PLATFORM_OutputTransistor((transistor_on != 0U) ? ENABLE : DISABLE);
}

/* 定义候选计数清零函数；需要时先记录未确认候选耗时。 */
static void Sensor_ResetCandidates(uint32_t now, uint8_t log_abort)
{
/* 仅在编译时打开 ABORT 日志功能时包含以下日志代码。 */
#if SENSOR_TIMING_LOG_ABORTED_CANDIDATE
    /* 若调用者要求记录且当前存在 ON 候选帧，则输出 ON_ABORT。 */
    if ((log_abort != 0U) && (g_sensor.on_count != 0U))
    {
        /* 记录 ON 候选中止事件名称及其上下文。 */
        Sensor_DebugTimingEvent("ON_ABORT",
                                g_sensor.last_distance_mm,
                                g_sensor.on_count,
                                /* 执行当前语句。 */
                                (uint32_t)(now - g_sensor.on_start_tick));
    }

    /* 若调用者要求记录且当前存在 OFF 候选帧，则输出 OFF_ABORT。 */
    if ((log_abort != 0U) && (g_sensor.off_count != 0U))
    {
        /* 记录 OFF 候选中止事件名称及其上下文。 */
        Sensor_DebugTimingEvent("OFF_ABORT",
                                g_sensor.last_distance_mm,
                                g_sensor.off_count,
                                /* 执行当前语句。 */
                                (uint32_t)(now - g_sensor.off_start_tick));
    }
/* 若 ABORT 日志未启用，以下两行仅用于抑制未使用参数告警。 */
#else
    /* 执行当前语句。 */
    (void)now;
    /* 执行当前语句。 */
    (void)log_abort;
/* 结束本组条件编译控制。 */
#endif

    /* 无论是否打印日志，最终都清零 ON 连续确认计数。 */
    g_sensor.on_count = 0U;
    /* 无论是否打印日志，最终都清零 OFF 连续确认计数。 */
    g_sensor.off_count = 0U;
}

/* 定义进入故障状态的统一处理函数。 */
static void Sensor_EnterFault(uint32_t now)
{
    /* 若已经处于故障状态则不重复执行，避免反复打印故障事件。 */
    if (g_sensor.state == SENSOR_STATE_FAULT)
    {
        /* 直接退出，保留当前故障输出。 */
        return;
    }

    /* 进入故障前清理候选计数，必要时输出 ABORT 日志。 */
    Sensor_ResetCandidates(now, 1U);
    /* 把业务状态切换为故障。 */
    g_sensor.state = SENSOR_STATE_FAULT;
    /* 立即按故障安全策略更新 Q1 输出。 */
    Sensor_ApplyOutput();

/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_TEST_ENABLE
    /* 测试模式下输出一次故障事件日志。 */
    USART2_DebugWriteString("EV=FAULT\r\n");
/* 结束本组条件编译控制。 */
#endif
}

/* 定义固定小数组的中位数函数，用于降低单帧 TOF 距离跳变对标定的影响。 */
static int16_t Sensor_GetMedian(const int16_t *values, uint8_t count)
{
    /* 在栈上创建排序副本；不改变原始采样数组。 */
    int16_t sorted[SENSOR_CAL_SAMPLE_COUNT];
    /* 插入排序时临时保存当前待插入值。 */
    int16_t temp;
    /* 声明外层循环索引。 */
    uint8_t i;
    /* 声明内层移动循环索引。 */
    uint8_t j;

    /* 把输入样本逐个复制到本地排序数组。 */
    for (i = 0U; i < count; i++)
    {
        /* 复制当前样本，后续只操作 sorted。 */
        sorted[i] = values[i];
    }

    /* 小数组插入排序，避免使用大内存或浮点。 */
    /* 从第二个元素开始执行插入排序。 */
    for (i = 1U; i < count; i++)
    {
        /* 取出当前待插入元素。 */
        temp = sorted[i];
        /* 初始化插入位置。 */
        j = i;

        /* 只要左侧元素更大，就向右移动，给 temp 腾出正确位置。 */
        while ((j > 0U) && (sorted[(uint8_t)(j - 1U)] > temp))
        {
            /* 把较大的左侧元素右移一格。 */
            sorted[j] = sorted[(uint8_t)(j - 1U)];
            /* 继续检查前一个元素。 */
            j--;
        }

        /* 将当前元素插入最终确定的位置。 */
        sorted[j] = temp;
    }

    /* 样本数为奇数，排序后正中间元素即为中位数。 */
    return sorted[count / 2U];
}

/* 定义临时 LED 指示设置函数。 */
static void Sensor_SetIndication(Indication_t indication, uint32_t now, uint32_t duration_ms)
{
    /* 保存当前优先显示的提示类型。 */
    g_sensor.indication = indication;
    /* 把当前时间加显示时长，得到该提示的截止时间。 */
    g_sensor.indication_until_tick = now + duration_ms;
}

/* 定义标定失败的统一退出与提示函数。 */
static void Sensor_CalibrationFail(uint32_t now, const char *reason)
{
    /* 停止当前标定流程，返回空闲阶段。 */
    g_sensor.cal_step = CAL_STEP_IDLE;
    /* 清空已经收集的标定样本数量。 */
    g_sensor.cal_sample_count = 0U;
    /* 设置 1200 ms 的标定失败 LED 提示。 */
    Sensor_SetIndication(INDICATION_CAL_FAIL, now, 1200U);

/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_TEST_ENABLE
    /* 先输出 CAL_FAIL 事件前缀。 */
    Sensor_DebugPrefix("CAL_FAIL,");
    /* 输出失败原因，例如跨度不足或采样超时。 */
    USART2_DebugWriteString(reason);
    /* 结束标定失败日志。 */
    USART2_DebugWriteString("\r\n");
/* 切换到条件编译的另一分支，用于关闭功能时保持编译告警为零。 */
#else
    /* 执行当前语句。 */
    (void)reason;
/* 结束本组条件编译控制。 */
#endif
}

/* 定义开始第一段或第二段标定采样的函数。 */
static void Sensor_StartCalibration(uint8_t point, uint32_t now)
{
    /* 新一段标定开始前清空上一段样本计数。 */
    g_sensor.cal_sample_count = 0U;
    /* 记录本段采样开始时刻，用于 2 秒采样超时。 */
    g_sensor.cal_step_start_tick = now;

    /* 根据传入 point 决定进入第一段还是第二段采样。 */
    if (point == 1U)
    {
        /* 切换到第一段采样状态。 */
        g_sensor.cal_step = CAL_STEP_CAPTURE_1;
/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_TEST_ENABLE
        /* 测试模式下记录第一段采样开始。 */
        USART2_DebugWriteString("EV=CAL1_START\r\n");
/* 结束本组条件编译控制。 */
#endif
    }
    /* point 不为 1 时按第二段标定处理。 */
    else
    {
        /* 切换到第二段采样状态。 */
        g_sensor.cal_step = CAL_STEP_CAPTURE_2;
/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_TEST_ENABLE
        /* 测试模式下记录第二段采样开始。 */
        USART2_DebugWriteString("EV=CAL2_START\r\n");
/* 结束本组条件编译控制。 */
#endif
    }
}

/* 定义两段标定完成后的有效性检查、设定点计算和初始输出处理。 */
static void Sensor_CompleteCalibration(uint32_t now)
{
    /* 保存两个标定点之间的绝对距离跨度。 */
    uint16_t span;
    /* 保存允许标定成功的最小跨度。 */
    uint16_t minimum_span;
    /* 用 32 位变量计算中点，避免两个 int16_t 相加溢出。 */
    int32_t midpoint;

    /* 计算两标定点的绝对间距。 */
    span = Sensor_AbsDiffU16(g_sensor.cal_point_1_mm, g_sensor.cal_point_2_mm);
    /* 最小跨度 = 双边迟滞宽度 + 额外安全裕量。 */
    minimum_span = (uint16_t)((2U * SENSOR_HYSTERESIS_MM) + SENSOR_CAL_EXTRA_GUARD_MM);

    /* 如果两点距离过近，迟滞窗口将无法稳定工作。 */
    if (span < minimum_span)
    {
        /* 以 SPAN 原因终止本次标定。 */
        Sensor_CalibrationFail(now, "SPAN");
        /* 标定失败后不再继续计算设定点。 */
        return;
    }

    /* 计算两个标定点的算术平均值，即最终距离设定点。 */
    midpoint = ((int32_t)g_sensor.cal_point_1_mm + (int32_t)g_sensor.cal_point_2_mm) / 2;
    /* 把 32 位中点转换回 int16_t 保存。 */
    g_sensor.setpoint_mm = (int16_t)midpoint;
    /* 标记当前已经拥有有效标定结果。 */
    g_sensor.calibrated = 1U;
    /* 标定状态机回到空闲，允许后续重新标定。 */
    g_sensor.cal_step = CAL_STEP_IDLE;

    /* 标定结束时从“遮光”逻辑起步，下一帧起再按 N 帧确认输出。 */
    /* 清除任何遗留的 ON/OFF 候选计数，不打印中止日志。 */
    Sensor_ResetCandidates(now, 0U);
    /* 标定后强制从遮光状态开始，避免立即依据单帧跳入触发。 */
    g_sensor.state = SENSOR_STATE_BLOCKED;
    /* 按当前 NO/NC 逻辑更新物理输出。 */
    Sensor_ApplyOutput();

    /* 显示 800 ms 的标定成功指示。 */
    Sensor_SetIndication(INDICATION_CAL_SUCCESS, now, 800U);
    /* 输出标定点、设定点和迟滞调试信息。 */
    Sensor_DebugCalibration("CAL_OK");
}

/* 定义向当前标定段追加一个有效距离样本的函数。 */
static void Sensor_AddCalibrationSample(int16_t distance_mm, uint32_t now)
{
    /* 防御性限制：样本已经满时不再写数组。 */
    if (g_sensor.cal_sample_count >= SENSOR_CAL_SAMPLE_COUNT)
    {
        /* 样本数量达到上限时直接退出。 */
        return;
    }

    /* 把距离写入当前样本槽位，然后样本计数加一。 */
    g_sensor.cal_samples[g_sensor.cal_sample_count++] = distance_mm;

    /* 未收满 7 帧时仅继续等待下一帧。 */
    if (g_sensor.cal_sample_count < SENSOR_CAL_SAMPLE_COUNT)
    {
        /* 保留当前采样状态，等待下一帧。 */
        return;
    }

    /* 当本段是第一段采样时，完成第一标定点。 */
    if (g_sensor.cal_step == CAL_STEP_CAPTURE_1)
    {
        /* 对第一段 7 个样本取中位数，得到 cal_point_1_mm。 */
        g_sensor.cal_point_1_mm = Sensor_GetMedian(g_sensor.cal_samples,
                                                    /* 执行当前语句。 */
                                                    SENSOR_CAL_SAMPLE_COUNT);
        /* 第一段结束后进入等待第二次按键阶段。 */
        g_sensor.cal_step = CAL_STEP_WAIT_2;
        /* 从此时开始计第二段按键等待超时。 */
        g_sensor.cal_step_start_tick = now;
        /* 输出第一段标定完成日志。 */
        Sensor_DebugCalibration("CAL1_OK");
    }
    /* 当本段是第二段采样时，完成第二标定点并进入总标定完成流程。 */
    else if (g_sensor.cal_step == CAL_STEP_CAPTURE_2)
    {
        /* 对第二段 7 个样本取中位数，得到 cal_point_2_mm。 */
        g_sensor.cal_point_2_mm = Sensor_GetMedian(g_sensor.cal_samples,
                                                    /* 执行当前语句。 */
                                                    SENSOR_CAL_SAMPLE_COUNT);
        /* 执行跨度检查、中点计算和标定完成后的输出初始化。 */
        Sensor_CompleteCalibration(now);
    }
}

/* 定义核心距离状态机：迟滞比较、连续帧确认、输出切换和响应时间记录均在此完成。 */
static void Sensor_ProcessDistance(int16_t distance_mm, uint32_t now)
{
    /* 声明上阈值变量，使用 32 位防止 setpoint+迟滞溢出。 */
    int32_t upper;
    /* 声明下阈值变量，使用 32 位防止 setpoint-迟滞溢出。 */
    int32_t lower;
    /* 声明本次候选确认耗时变量。 */
    uint32_t duration;

    /* 计算触发上阈值 = 设定点 + 迟滞。 */
    upper = (int32_t)g_sensor.setpoint_mm + (int32_t)SENSOR_HYSTERESIS_MM;
    /* 计算遮光下阈值 = 设定点 - 迟滞。 */
    lower = (int32_t)g_sensor.setpoint_mm - (int32_t)SENSOR_HYSTERESIS_MM;

    /* 仅在当前稳定遮光时，才检测是否应进入/确认触发。 */
    if (g_sensor.state == SENSOR_STATE_BLOCKED)
    {
        /* 当前准备做 ON 判断，OFF 候选计数必须清零。 */
        g_sensor.off_count = 0U;

        /* 距离达到上阈值，进入 ON 候选确认区。 */
        if ((int32_t)distance_mm >= upper)
        {
            /* 若这是本轮第一个 ON 候选帧，则记录起始时刻。 */
            if (g_sensor.on_count == 0U)
            {
                /* 保存 ON 候选开始时间，用于最终输出确认耗时。 */
                g_sensor.on_start_tick = now;
            }

            /* 未达到最大确认数前继续累加 ON 连续帧。 */
            if (g_sensor.on_count < SENSOR_ON_CONFIRM_FRAMES)
            {
                /* 记录又获得一帧连续满足上阈值的有效测量。 */
                g_sensor.on_count++;
            }

            /* 当 ON 连续帧达到设定数量时，正式切换为触发。 */
            if (g_sensor.on_count >= SENSOR_ON_CONFIRM_FRAMES)
            {
                /* 计算从第一 ON 候选帧到第 N 帧确认的经过时间。 */
                duration = (uint32_t)(now - g_sensor.on_start_tick);
                /* 状态切换为稳定触发。 */
                g_sensor.state = SENSOR_STATE_TRIGGERED;
                /* 确认完成后清空 ON 计数，为下一次状态循环准备。 */
                g_sensor.on_count = 0U;
                /* 按 NO/NC 模式立即更新 Q1 输出。 */
                Sensor_ApplyOutput();
                /* 输出 ON 事件及距离、确认帧数、确认耗时。 */
                Sensor_DebugTimingEvent("ON",
                                        distance_mm,
                                        SENSOR_ON_CONFIRM_FRAMES,
                                        /* 执行当前语句。 */
                                        duration);
            }
        }
        /* 若距离未达到上阈值（包括迟滞区），本轮 ON 连续性被打断。 */
        else
        {
            /* 迟滞区也不保留候选计数，保证“连续有效帧”定义严格成立。 */
/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_LOG_ABORTED_CANDIDATE
            /* 若此前已有 ON 候选，则记录 ON_ABORT 事件。 */
            if (g_sensor.on_count != 0U)
            {
                /* 输出 ON 候选被打断时的距离、累计帧数和耗时。 */
                Sensor_DebugTimingEvent("ON_ABORT",
                                        distance_mm,
                                        g_sensor.on_count,
                                        /* 执行当前语句。 */
                                        (uint32_t)(now - g_sensor.on_start_tick));
            }
/* 结束本组条件编译控制。 */
#endif
            /* 清零 ON 候选帧数，确保确认必须严格连续。 */
            g_sensor.on_count = 0U;
        }
    }
    /* 仅在当前稳定触发时，才检测是否应进入/确认遮光。 */
    else if (g_sensor.state == SENSOR_STATE_TRIGGERED)
    {
        /* 当前准备做 OFF 判断，ON 候选计数必须清零。 */
        g_sensor.on_count = 0U;

        /* 距离低于或等于下阈值，进入 OFF 候选确认区。 */
        if ((int32_t)distance_mm <= lower)
        {
            /* 若这是本轮第一个 OFF 候选帧，则记录起始时刻。 */
            if (g_sensor.off_count == 0U)
            {
                /* 保存 OFF 候选开始时间，用于最终遮光确认耗时。 */
                g_sensor.off_start_tick = now;
            }

            /* 未达到最大确认数前继续累加 OFF 连续帧。 */
            if (g_sensor.off_count < SENSOR_OFF_CONFIRM_FRAMES)
            {
                /* 记录又获得一帧连续满足下阈值的有效测量。 */
                g_sensor.off_count++;
            }

            /* 当 OFF 连续帧达到设定数量时，正式切换为遮光。 */
            if (g_sensor.off_count >= SENSOR_OFF_CONFIRM_FRAMES)
            {
                /* 计算从第一 OFF 候选帧到第 N 帧确认的经过时间。 */
                duration = (uint32_t)(now - g_sensor.off_start_tick);
                /* 状态切换为稳定遮光。 */
                g_sensor.state = SENSOR_STATE_BLOCKED;
                /* 确认完成后清空 OFF 计数。 */
                g_sensor.off_count = 0U;
                /* 按 NO/NC 模式立即更新 Q1 输出。 */
                Sensor_ApplyOutput();
                /* 输出 OFF 事件及距离、确认帧数、确认耗时。 */
                Sensor_DebugTimingEvent("OFF",
                                        distance_mm,
                                        SENSOR_OFF_CONFIRM_FRAMES,
                                        /* 执行当前语句。 */
                                        duration);
            }
        }
        /* 若距离未落入下阈值区（包括迟滞区），本轮 OFF 连续性被打断。 */
        else
        {
/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_LOG_ABORTED_CANDIDATE
            /* 若此前已有 OFF 候选，则记录 OFF_ABORT 事件。 */
            if (g_sensor.off_count != 0U)
            {
                /* 输出 OFF 候选被打断时的距离、累计帧数和耗时。 */
                Sensor_DebugTimingEvent("OFF_ABORT",
                                        distance_mm,
                                        g_sensor.off_count,
                                        /* 执行当前语句。 */
                                        (uint32_t)(now - g_sensor.off_start_tick));
            }
/* 结束本组条件编译控制。 */
#endif
            /* 清零 OFF 候选帧数，确保确认必须严格连续。 */
            g_sensor.off_count = 0U;
        }
    }
}

/* 定义接收一帧已解析 TOF 数据后的业务分发函数。 */
static void Sensor_HandleFrame(const TOF_Frame_t *frame)
{
    /* 声明本帧对应的毫秒时间戳。 */
    uint32_t now;

    /* 防御性检查，禁止处理空帧指针。 */
    if (frame == 0)
    {
        /* 空帧直接返回。 */
        return;
    }

    /* 取出帧完成时刻作为本帧的业务时间基准。 */
    now = frame->tick_ms;
    /* 更新最近距离，供日志、状态观察和候选中止记录使用。 */
    g_sensor.last_distance_mm = frame->distance_mm;
    /* 更新最近置信度。 */
    g_sensor.last_confidence = frame->confidence;

    /* 先执行置信度和距离范围过滤。 */
    if (Sensor_IsMeasurementValid(frame) == 0U)
    {
        /* 低置信度/越界帧不是“遮光帧”，只中断当前连续确认。 */
        /* 无效帧只打断候选确认，不把它错误解释成遮光。 */
        Sensor_ResetCandidates(now, 1U);
        /* 无效帧不参与标定，也不继续距离状态机。 */
        return;
    }

    /* 记录最近一次真正可信测量的到达时刻。 */
    g_sensor.last_good_measurement_tick = now;

    /* 若当前正处于任一标定采样阶段，优先将本帧用于收集样本。 */
    if ((g_sensor.cal_step == CAL_STEP_CAPTURE_1) ||
        (g_sensor.cal_step == CAL_STEP_CAPTURE_2))
    {
        /* 将本帧距离加入 7 点标定样本数组。 */
        Sensor_AddCalibrationSample(frame->distance_mm, now);
        /* 标定采样期间不执行正常触发/遮光判定。 */
        return;
    }

    /* 未标定状态下不允许对距离做输出控制。 */
    if (g_sensor.calibrated == 0U)
    {
        /* 直接返回，等待用户完成两段标定。 */
        return;
    }

    /* 通信/测量恢复后，从 BLOCKED 重建 N 帧确认，禁止一帧直接跳变。 */
    /* 若此前因超时进入故障，而当前收到可信帧，则执行通信恢复。 */
    if (g_sensor.state == SENSOR_STATE_FAULT)
    {
        /* 恢复时清除候选计数，但不输出 ABORT 日志。 */
        Sensor_ResetCandidates(now, 0U);
        /* 恢复后固定先回到遮光稳定状态，再重新按 N 帧确认触发。 */
        g_sensor.state = SENSOR_STATE_BLOCKED;
        /* 更新恢复后的物理输出。 */
        Sensor_ApplyOutput();
/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_TEST_ENABLE
        /* 测试模式下记录通信恢复事件。 */
        USART2_DebugWriteString("EV=RECOVER\r\n");
/* 结束本组条件编译控制。 */
#endif
    }

    /* 将可信距离交给迟滞与连续确认状态机。 */
    Sensor_ProcessDistance(frame->distance_mm, now);
}

/* 定义所有非阻塞超时检查：标定阶段超时、测量超时、协议帧超时。 */
static void Sensor_ServiceTimeouts(uint32_t now)
{
    /* 若正在第一段或第二段采样，则检查采样时间是否超过 2 秒。 */
    if ((g_sensor.cal_step == CAL_STEP_CAPTURE_1) ||
        (g_sensor.cal_step == CAL_STEP_CAPTURE_2))
    {
        /* 计算当前采样段已经持续多久。 */
        if (Sensor_TimeElapsed(now,
                               g_sensor.cal_step_start_tick,
                               SENSOR_CAL_CAPTURE_TIMEOUT_MS) != 0U)
        {
            /* 以 CAPTURE_TO 原因终止超时的采样段。 */
            Sensor_CalibrationFail(now, "CAPTURE_TO");
        }
    }
    /* 若已完成第一段，检查等待第二次按键是否超过 30 秒。 */
    else if (g_sensor.cal_step == CAL_STEP_WAIT_2)
    {
        /* 计算第二段等待时间。 */
        if (Sensor_TimeElapsed(now,
                               g_sensor.cal_step_start_tick,
                               SENSOR_CAL_WAIT_SECOND_TIMEOUT_MS) != 0U)
        {
            /* 以 WAIT2_TO 原因终止本次标定。 */
            Sensor_CalibrationFail(now, "WAIT2_TO");
        }
    }

    /* 只有已经标定后的产品才启用 TOF 通信/测量故障判断。 */
    if (g_sensor.calibrated != 0U)
    {
        /* 检查最近可信测量是否超过业务超时。 */
        if (Sensor_TimeElapsed(now,
                               g_sensor.last_good_measurement_tick,
                               SENSOR_MEASUREMENT_TIMEOUT_MS) != 0U)
        {
            /* 超过 200 ms 无可信测量时进入故障安全状态。 */
            Sensor_EnterFault(now);
        }
        /* 若可信测量仍新鲜，再检查是否长时间没有完整有效协议帧。 */
        else if (Sensor_TimeElapsed(now,
                                    g_tof_last_valid_tick_ms,
                                    SENSOR_FRAME_TIMEOUT_MS) != 0U)
        {
            /* 超过 150 ms 无有效帧时同样进入故障状态。 */
            Sensor_EnterFault(now);
        }
    }
}

/* 定义非阻塞 LED 显示服务函数，主循环持续调用。 */
static void Sensor_LEDService(uint32_t now)
{
    /* 声明慢闪状态变量。 */
    uint8_t blink_slow;
    /* 声明快闪状态变量。 */
    uint8_t blink_fast;

    /* 每 500 ms 翻转一次，形成约 1 Hz 的慢闪。 */
    blink_slow = (((now / 500U) & 1U) != 0U) ? 1U : 0U;
    /* 每 125 ms 翻转一次，形成约 4 Hz 的快闪。 */
    blink_fast = (((now / 125U) & 1U) != 0U) ? 1U : 0U;

    /* 临时提示存在时，其优先级高于正常状态灯效。 */
    if (g_sensor.indication != INDICATION_NONE)
    {
        /* 若当前时间达到临时提示截止时刻，则取消临时提示。 */
        if (Sensor_TimeReached(now, g_sensor.indication_until_tick) != 0U)
        {
            /* 恢复无临时指示状态，后续进入正常灯效逻辑。 */
            g_sensor.indication = INDICATION_NONE;
        }
        /* 前面条件均不成立时执行此备用分支。 */
        else
        {
            /* 标定成功提示：绿灯常亮、红灯熄灭。 */
            if (g_sensor.indication == INDICATION_CAL_SUCCESS)
            {
                /* 点亮绿色 LED。 */
                PLATFORM_LED_Enable(LED1, ENABLE);
                /* 关闭红色 LED。 */
                PLATFORM_LED_Enable(LED2, DISABLE);
                /* 临时提示已处理，直接返回，避免被普通状态覆盖。 */
                return;
            }

            /* 标定失败提示：红灯快闪、绿灯熄灭。 */
            if (g_sensor.indication == INDICATION_CAL_FAIL)
            {
                /* 关闭绿色 LED。 */
                PLATFORM_LED_Enable(LED1, DISABLE);
                /* 根据快闪相位驱动红色 LED。 */
                PLATFORM_LED_Enable(LED2, (blink_fast != 0U) ? ENABLE : DISABLE);
                /* 临时提示已处理，直接返回。 */
                return;
            }

            /* 模式切换提示：两灯同步快闪。 */
            if (g_sensor.indication == INDICATION_MODE_CHANGED)
            {
                /* 按快闪相位驱动绿色 LED。 */
                PLATFORM_LED_Enable(LED1, (blink_fast != 0U) ? ENABLE : DISABLE);
                /* 按快闪相位驱动红色 LED。 */
                PLATFORM_LED_Enable(LED2, (blink_fast != 0U) ? ENABLE : DISABLE);
                /* 临时提示已处理，直接返回。 */
                return;
            }
        }
    }

    /* 第一段标定采样时：绿灯慢闪，红灯熄灭。 */
    if (g_sensor.cal_step == CAL_STEP_CAPTURE_1)
    {
        /* 按慢闪相位驱动绿色 LED。 */
        PLATFORM_LED_Enable(LED1, (blink_slow != 0U) ? ENABLE : DISABLE);
        /* 关闭红色 LED。 */
        PLATFORM_LED_Enable(LED2, DISABLE);
        /* 该标定状态灯效已完成，直接返回。 */
        return;
    }

    /* 等待第二次按键时：绿灯快闪，红灯熄灭。 */
    if (g_sensor.cal_step == CAL_STEP_WAIT_2)
    {
        /* 按快闪相位驱动绿色 LED。 */
        PLATFORM_LED_Enable(LED1, (blink_fast != 0U) ? ENABLE : DISABLE);
        /* 关闭红色 LED。 */
        PLATFORM_LED_Enable(LED2, DISABLE);
        /* 该标定状态灯效已完成，直接返回。 */
        return;
    }

    /* 第二段标定采样时：绿灯和红灯同步快闪。 */
    if (g_sensor.cal_step == CAL_STEP_CAPTURE_2)
    {
        /* 按快闪相位驱动绿色 LED。 */
        PLATFORM_LED_Enable(LED1, (blink_fast != 0U) ? ENABLE : DISABLE);
        /* 按快闪相位驱动红色 LED。 */
        PLATFORM_LED_Enable(LED2, (blink_fast != 0U) ? ENABLE : DISABLE);
        /* 该标定状态灯效已完成，直接返回。 */
        return;
    }

    /* 未标定或故障时：绿灯熄灭、红灯慢闪。 */
    if ((g_sensor.state == SENSOR_STATE_UNCALIBRATED) ||
        (g_sensor.state == SENSOR_STATE_FAULT))
    {
        /* 关闭绿色 LED。 */
        PLATFORM_LED_Enable(LED1, DISABLE);
        /* 按慢闪相位驱动红色 LED。 */
        PLATFORM_LED_Enable(LED2, (blink_slow != 0U) ? ENABLE : DISABLE);
        /* 故障/未标定灯效已完成，直接返回。 */
        return;
    }

    /* 正常已标定状态下，依据稳定业务状态显示绿/红灯。 */
    if (g_sensor.state == SENSOR_STATE_TRIGGERED)
    {
        /* 触发稳定时点亮绿灯。 */
        PLATFORM_LED_Enable(LED1, ENABLE);
        /* 触发稳定时关闭红灯。 */
        PLATFORM_LED_Enable(LED2, DISABLE);
    }
    /* 除触发外即遮光稳定状态。 */
    else
    {
        /* 遮光稳定时关闭绿灯。 */
        PLATFORM_LED_Enable(LED1, DISABLE);
        /* 遮光稳定时点亮红灯。 */
        PLATFORM_LED_Enable(LED2, ENABLE);
    }
}

/* 定义按键消抖对象的初始化函数。 */
static void KeyDebounce_Init(KeyDebounce_t *key, PLATFORM_Key_t key_id, uint32_t now)
{
    /* 读取当前硬件按键电平作为初始 raw 状态。 */
    key->raw = PLATFORM_KeyIsPressed(key_id);
    /* 初始化时直接把该状态作为稳定状态，避免上电误报按下沿。 */
    key->stable = key->raw;
    /* 记录初始化时刻，作为后续消抖计时基准。 */
    key->raw_change_tick = now;
}

/* 返回 1 代表一次“确认后的按下沿”。 */
/* 定义获取“确认后的按下沿”事件函数；每次主循环调用一次。 */
static uint8_t KeyDebounce_GetPressEvent(KeyDebounce_t *key,
                                         PLATFORM_Key_t key_id,
                                         /* 声明局部变量，为当前函数的判断、计算或状态更新保存临时数据。 */
                                         uint32_t now)
{
    /* 声明本次读取的原始按键电平。 */
    uint8_t raw;

    /* 从对应 GPIO 读取当前按键是否被按下。 */
    raw = PLATFORM_KeyIsPressed(key_id);

    /* 若当前原始电平与上次不同，说明可能发生了抖动或真实边沿。 */
    if (raw != key->raw)
    {
        /* 更新 raw 电平记录。 */
        key->raw = raw;
        /* 从本次变化开始重新计 30 ms 消抖时间。 */
        key->raw_change_tick = now;
    }

    /* 只有 raw 与 stable 不同且新 raw 已持续稳定 30 ms，才认可这次状态变化。 */
    if ((raw != key->stable) &&
        (Sensor_TimeElapsed(now, key->raw_change_tick, SENSOR_KEY_DEBOUNCE_MS) != 0U))
    {
        /* 提交新的稳定按键状态。 */
        key->stable = raw;

        /* 只有稳定状态变为按下，才向上层产生一次 press 事件；松开不产生事件。 */
        if (key->stable != 0U)
        {
            /* 返回 1，通知调用者本轮确认到按下沿。 */
            return 1U;
        }
    }

    /* 没有确认按下事件时返回 0。 */
    return 0U;
}

/* 定义按键业务服务：按键 1 控制两段标定，按键 2 切换 NO/NC。 */
static void Sensor_KeyService(uint32_t now)
{
    /* 轮询并消抖按键 1；确认按下时进入标定状态机。 */
    if (KeyDebounce_GetPressEvent(&g_key1, PLATFORM_KEY1, now) != 0U)
    {
        /* 空闲状态下，按键 1 启动第一段标定。 */
        if (g_sensor.cal_step == CAL_STEP_IDLE)
        {
            /* 开始采集第一标定点。 */
            Sensor_StartCalibration(1U, now);
        }
        /* 第一段已完成且等待第二段时，按键 1 启动第二段。 */
        else if (g_sensor.cal_step == CAL_STEP_WAIT_2)
        {
            /* 开始采集第二标定点。 */
            Sensor_StartCalibration(2U, now);
        }
        /* 两段正在取样时忽略重复按键，防止半程重新清零。 */
    }

    /* 轮询并消抖按键 2；确认按下时切换输出逻辑。 */
    if (KeyDebounce_GetPressEvent(&g_key2, PLATFORM_KEY2, now) != 0U)
    {
        /* 把当前 NO/NC 枚举在两种模式之间翻转。 */
        g_sensor.output_mode = (g_sensor.output_mode == SENSOR_OUTPUT_NO) ?
                               /* 执行当前语句。 */
                               SENSOR_OUTPUT_NC : SENSOR_OUTPUT_NO;
        /* 模式切换应立即刷新 Q1 输出，不能等待下一个距离帧。 */
        Sensor_ApplyOutput();
        /* 显示 350 ms 的模式切换双灯闪烁提示。 */
        Sensor_SetIndication(INDICATION_MODE_CHANGED, now, 350U);
        /* 输出当前新模式调试日志。 */
        Sensor_DebugMode();
    }
}

/* 定义传感器应用状态机的上电初始化函数。 */
static void Sensor_AppInit(void)
{
    /* 声明初始化使用的当前毫秒 tick。 */
    uint32_t now;

    /* 获取平台当前时间，作为各状态机起点。 */
    now = PLATFORM_GetTickMs();

    /* 上电进入未标定状态。 */
    g_sensor.state = SENSOR_STATE_UNCALIBRATED;
    /* 根据默认宏设置初始 NO/NC 模式。 */
    g_sensor.output_mode = (SENSOR_DEFAULT_OUTPUT_MODE_NC != 0U) ?
                           /* 执行当前语句。 */
                           SENSOR_OUTPUT_NC : SENSOR_OUTPUT_NO;
    /* 标定子状态机从空闲开始。 */
    g_sensor.cal_step = CAL_STEP_IDLE;
    /* 明确标记上电尚未标定。 */
    g_sensor.calibrated = 0U;
    /* 初始化最近可信测量时间，避免刚上电立刻触发超时。 */
    g_sensor.last_good_measurement_tick = now;
    /* 初始化为无临时 LED 指示。 */
    g_sensor.indication = INDICATION_NONE;

    /* 初始化按键 1 消抖状态。 */
    KeyDebounce_Init(&g_key1, PLATFORM_KEY1, now);
    /* 初始化按键 2 消抖状态。 */
    KeyDebounce_Init(&g_key2, PLATFORM_KEY2, now);

    /* 按未标定输出策略写入 Q1。 */
    Sensor_ApplyOutput();
}

/* 主程序入口：初始化硬件和通信，然后永久执行非阻塞任务循环。 */
int main(void)
{
    /* 声明一个 TOF 帧局部缓存，主循环从接收队列逐帧取出。 */
    TOF_Frame_t frame;      //== 一帧数据缓存
    
    /* 声明主循环当前毫秒 tick。 */
    uint32_t now;           //== 当前ms滴答

    /* 初始化系统时钟、GPIO、LED、按键、USART2 调试口及 SysTick 等平台资源。 */
    PLATFORM_Init();        //== 外设初始化
    
    /* 初始化 USART1 TOF 通信，并开启 RXNE/IDLE 接收与帧解析机制。 */
    TOF_CommInit(TOF_DEFAULT_BAUDRATE);        //== 开启串口1串口2，打开一次空闲接收
    
    /* 初始化业务状态机、按键消抖与未标定输出。 */
    Sensor_AppInit();       //== 上电初始化状态机

/* 开始条件编译分支：仅当对应测试/日志宏启用时才编译以下代码。 */
#if SENSOR_TIMING_TEST_ENABLE
    /* 测试模式下打印固件已就绪标识。 */
    USART2_DebugWriteString("YC02_TOF01_SENSOR_READY\r\n");
/* 结束本组条件编译控制。 */
#endif

    /* 进入永久主循环；所有业务均采用轮询服务方式，不使用阻塞延时。 */
    while (1)
    {
        /* 优先级 1：完成帧立即解析并重新打开 USART1 接收。 */
        /* 优先处理串口接收端已经完成的一帧，并尽快恢复下一帧接收。 */
        TOF_Service();

        /* 优先级 2：将有效帧交给商业判定状态机。 */
        /* 只要队列中仍有完整有效帧，就持续弹出并交给业务状态机。 */
        while (TOF_PopValidFrame(&frame) != 0U)
        {
            /* 对当前 TOF 帧执行有效性检查、标定收样或距离状态判定。 */
            Sensor_HandleFrame(&frame);
        }

        /* 读取当前毫秒 tick，供按键、超时和 LED 服务共用。 */
        now = PLATFORM_GetTickMs();

        /* 优先级 3：按键、超时、灯光均不阻塞。 */
        /* 处理按键消抖后的业务事件。 */
        Sensor_KeyService(now);
        
        /* 处理标定、测量和协议帧的所有超时。 */
        Sensor_ServiceTimeouts(now);
        
        /* 刷新当前应显示的 LED 灯效。 */
        Sensor_LEDService(now);
    }
}
