#include "drv_can.h"

//----------变量---------------------------

static CAN_BusState can1_state; // 各个FDCAN的总线状态对象们
static CAN_BusState can2_state;
static CAN_BusState can3_state;

//------------宏定义-------------------------

#define CAN_STD_FILTER_INDEX 0U // 按照CubeMX的StdFilter=1设定，使用第一个标准器槽位
#define CAN_STD_ID_MAX 0x77U    // CAN标准帧ID的最大值,用于校验

#define CAN_HAL_RX_SCRATCH_BYTES 64U // 本地草稿区，先复制数据再返回给调用方检查。这个零食数字组覆盖DLC表的最大长度，避免超长报文

//------------函数定义-----------------------

/**
 * @brief 通过句柄地址匹配相应总线状态对象
 *
 * @param hfdcan 句柄
 * @return CAN_BusState* 总线状态对象的地址
 */
static CAN_BusState *CAN_FindBus(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) // 按句柄地址匹配
    {
        return &can1_state; // 匹配相应总线状态对象
    }
    else if (hfdcan == &hfdcan2)
    {
        return &can2_state;
    }
    else if (hfdcan == &hfdcan3)
    {
        return &can3_state;
    }
    else
    {
        return NULL;
    }
}

/**
 * @brief 配置标准ID接收规则、保存应用回调、启动FDCAN和开启接收中断通知的函数
 *
 * @param hfdcan FDCAN句柄
 * @param first_id 精确匹配时的对应ID 范围匹配时的下限，如：0x201
 * @param last_id 精确匹配时的对应ID 范围匹配时的上限，如：0x204
 * @param callback 回调函数
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_STD_StartRx(FDCAN_HandleTypeDef *hfdcan, uint16_t first_id, uint16_t last_id, CAN_Call_Back callback)
{
    CAN_BusState *bus = CAN_FindBus(hfdcan); // 用于保存根据传入的FDCAN句柄找到的对应总线状态对象
    HAL_StatusTypeDef status;                // 用于保存HAL函数返回的执行结果

    // 配置过滤器
    FDCAN_FilterTypeDef filter =
        {
            .IdType = FDCAN_STANDARD_ID,             // 设置规则为标准帧
            .FilterIndex = CAN_STD_FILTER_INDEX,     // 使用槽位
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, // FIFO
    };

    // 检查传进参数是否有误，只要传入未知的句柄/没有提供回调处理函数/接受范围写反了/上限超出了就返回ERROR
    if (bus == NULL || callback == NULL || first_id > last_id || last_id > CAN_STD_ID_MAX)
    {
        return HAL_ERROR;
    }
    // 检查CubeMX的配置是否满足后续接收要求。如果外设未处于初始化完成的就绪状态/已分配范围小于准备给的分配范围/FIFO0未分配接收元素
    if (HAL_FDCAN_GetState(hfdcan) != HAL_FDCAN_STATE_READY || hfdcan->Init.StdFiltersNbr <= CAN_STD_FILTER_INDEX || hfdcan->Init.RxFifo0ElmtsNbr == 0U)
    {
        return HAL_ERROR;
    }

    // 配置filterID
    filter.FilterID1 = first_id; // 精确匹配下的id/范围匹配下的范围下线，如0x201
    if (first_id == last_id)
    {
        filter.FilterType = FDCAN_FILTER_MASK; // 掩码模式，要求精确匹配
        filter.FilterID2 = CAN_STD_ID_MAX;     // 掩码模式下，FilterID2表示哪些id位需要参与比较
    }
    else
    {
        filter.FilterType = FDCAN_FILTER_RANGE; // 范围模式
        filter.FilterID2 = last_id;             // 范围模式下，FilterID2表示接受范围的上限
    }

    status = HAL_FDCAN_ConfigFilter(hfdcan, &filter); // 检查过滤器配置的配置结果
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_FDCAN_ConfigGlobalFilter(
        hfdcan,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE); // 配置全局过滤规则，决定没有命中ID规则的报文和远程帧怎么处理
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_FDCAN_ConfigInterruptLines(
        hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        FDCAN_INTERRUPT_LINE0); // 把FIFO0收到新报文这个时间分配到FDCAN的中断线IT0上
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_FDCAN_Start(hfdcan); // 启动FDCAN外设
    if (status != HAL_OK)
    {
        return status;
    }

    bus->receive = callback; // 绑定回调函数

    status = HAL_FDCAN_ActivateNotification(
        hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0U); // 开启FIFO0收到新报文的中断通知
    if (status != HAL_OK)
    {
        bus->receive = NULL;
        HAL_FDCAN_Stop(hfdcan); // 停止FDCAN外设
        return status;
    }

    return status;
}

/**
 * @brief 单次发送
 *
 * @param hfdcan fdcan句柄
 * @param id 发送给的ID
 * @param data 发送的数据
 * @param length 数据长度
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef CAN_STD_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, const uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef header =
        {
            .Identifier = id,                         // 设置值发送报文的ID
            .IdType = FDCAN_STANDARD_ID,              // 选择ID格式为标准ID格式
            .TxFrameType = FDCAN_DATA_FRAME,          // 选择帧类型为数据帧
            .DataLength = length,                     // 设置载荷长度
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,  // 设置ESI错误状态指示字段为error_active
            .BitRateSwitch = FDCAN_BRS_OFF,           // 设置BRS数据阶段速率切换为关闭
            .FDFormat = FDCAN_CLASSIC_CAN,            // 设置这一帧使用经典CAN格式
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS, // 这帧发送完成后不往TxEventFIFO写入发送事件记录
            .MessageMarker = 0U,                      // 关闭事件记录
    }; // 配置帧头
    uint8_t payload[CAN_CLASSIC_MAX_DATA_BYTES] = {0}; // 创建一个数字组在用来存放准备发送的数据并把所有字节初始化为0

    if (CAN_FindBus(hfdcan) == NULL || id > CAN_STD_ID_MAX || length > sizeof(payload) || (length != 0U && data == NULL)) // 发送前检查参数是否有效
    {
        return HAL_ERROR;
    }

    if (length != 0U)
    {
        memcpy(payload, data, length); // 写入
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, payload);
}

/**
 * @brief 接收中断函数
 *
 * @param hfdcan 句柄
 * @param events HAL直接传入的事件标志集合
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t events)
{
    CAN_BusState *bus = CAN_FindBus(hfdcan);    // 找到当前的总线状态对象
    uint8_t raw_data[CAN_HAL_RX_SCRATCH_BYTES]; // 准备一个临时接收数组

    if (bus == NULL || bus->receive == NULL || (events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return; // 退出函数
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U) // 只要FIFO0里还有尚未读取的报文就继续执行循环体
    {
        Struct_CAN_Rx_Buffer frame = {0};

        // 先尝试从FIFO0中读取一帧，再检查读出的报文是否符合要求
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &frame.Header, raw_data) != HAL_OK)
        {
            break; // 结束整个while循环，本次停止读取
        }
        // 检查这一帧的格式和长度
        if (frame.Header.FDFormat != FDCAN_CLASSIC_CAN || frame.Header.IdType != FDCAN_STANDARD_ID || frame.Header.RxFrameType != FDCAN_DATA_FRAME || frame.Header.DataLength > sizeof(frame.Data))
        {
            continue; // 结束当前这一轮，继续检查队列并读取下一帧
        }

        memcpy(frame.Data, raw_data, frame.Header.DataLength); // 填充数据
        bus->receive(&frame);                                  // 调用回调函数
    }
}
