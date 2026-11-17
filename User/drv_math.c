#include "drv_math.h"

//-----------函数------------------------
/**
 * @brief   16位大小端转换，原地互换
 * @param   Address 待转换数据的地址
 * @retval  None
 * @note    16位=两个字节，一个16位数由高字节、低字节组成。例：数字0x1234。逻辑上高字节0x12，低字节0x34
 * @note    大端：内存先存高字节，高到低 小端：内存先存低字节，低到高 大小端转换：交换正反位置
 */
void Math_Endian_Reverse_16(void *Address)
{
    uint8_t *temp_address_8 = (uint8_t *)Address;                  // 单独读取第0字节，第1字节的地址
    uint16_t *temp_address_16 = (uint16_t *)Address;               // 按16位整数访问读取存放的起始地址
    *temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1]; // 读取地址拿到字节大小端转换后放回原地址
}

/**
 * @brief   16位大小端转换，结果拷贝到目标地址
 * @param   Source 源数据地址
 * @param   Destination 目标存储地址
 * @retval  None
 */
void Math_Endian_Reverse_16_Copy(void *Source, void *Destination)
{
    uint8_t *temp_source, *temp_destination;
    temp_source = (uint8_t *)Source;
    temp_destination = (uint8_t *)Destination;

    temp_destination[0] = temp_source[1];
    temp_destination[1] = temp_source[0];
}

/**
 * @brief   32位大小端互换，原地修改
 * @param   Address 待转换数据的地址
 * @retval  None
 */
void Math_Endian_Reverse_32(void *Address)
{
    uint8_t *temp_address_8 = (uint8_t *)Address;
    uint32_t *temp_address_32 = (uint32_t *)Address;
    *temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];
}

/**
 * @brief   32位大小端互换，结果拷贝到目标地址
 * @param   Source 源数据地址
 * @param   Destination 目标存储地址
 * @retval  None
 */
void Math_Endian_Reverse_32_Copy(void *Source, void *Destination)
{
    uint8_t *temp_source, *temp_destination;
    temp_source = (uint8_t *)Source;
    temp_destination = (uint8_t *)Destination;

    temp_destination[0] = temp_source[3];
    temp_destination[1] = temp_source[2];
    temp_destination[2] = temp_source[1];
    temp_destination[3] = temp_source[0];
}

/**
 * @brief   16位求和
 * @param   Address 起始地址
 * @param   Length 被加的数据的数量
 * @retval  uint16_t的求和结果
 * @note    接收**uint16_t 类型数组首地址**，把数组里Length个 16 位无符号整数全部累加，返回总和
 */
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 32位求和
 * @param Address 起始地址
 * @param Length 被加的数据的数量
 * @retval uint32_t 求和结果
 */
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief sinc 函数实现
 * @param x 输入
 * @retval float 输出
 */
float Math_Sinc(float x)
{
    if (Math_Abs(x) <= 2.0f * FLT_EPSILON) // 分母为0则按极限求法
    {
        return (1.0f);
    }

    return (sin(x) / x);
}
