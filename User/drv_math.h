#if !defined(__DRV_MATH_H)
#define __DRV_MATH_H

//-----------includes--------------------
#include "stdint.h"
#include "limits.h"
#include "math.h"
#include "float.h"

//-----------宏定义-----------------------
#define PI (3.14159265f) // 圆周率

/**
 * @brief 限幅宏
 * @param x 指向被限幅变量的指针
 * @param Min 最小值
 * @param Max 最大值
 * @retval 无
 * @note 参数会被求值，注意传入的是指针
 */
#define Math_Constrain(x, Min, Max) \
    do                              \
    {                               \
        if (*(x) < (Min))           \
        {                           \
            *(x) = (Min);           \
        }                           \
        else if (*(x) > (Max))      \
        {                           \
            *(x) = (Max);           \
        }                           \
    } while (0) // \ 表示这一行还在延续

/**
 * @brief 求绝对值宏
 * @param x 输入值
 * @retval 绝对值
 * @note 参数会被求值两次，不要传入带副作用的表达式，如 Math_Abs(i++)
 */
#define Math_Abs(x) (((x) > 0) ? (x) : -(x))

//-----------函数声明---------------------
/**
 * @brief   16位大小端转换，原地互换
 * @param   Address 待转换数据的地址
 * @retval  None
 * @note    16位=两个字节，一个16位数由高字节、低字节组成。例：数字0x1234。逻辑上高字节0x12，低字节0x34
 * @note    大端：内存先存高字节，高到低 小端：内存先存低字节，低到高 大小端转换：交换正反位置
 */
void Math_Endian_Reverse_16(void *Address);

/**
 * @brief   16位大小端转换，结果拷贝到目标地址
 * @param   Source 源数据地址
 * @param   Destination 目标存储地址
 * @retval  None
 */
void Math_Endian_Reverse_16_Copy(void *Source, void *Destination);

/**
 * @brief   32位大小端互换，原地修改
 * @param   Address 待转换数据的地址
 * @retval  None
 */
void Math_Endian_Reverse_32(void *Address);

/**
 * @brief   32位大小端互换，结果拷贝到目标地址
 * @param   Source 源数据地址
 * @param   Destination 目标存储地址
 * @retval  None
 */
void Math_Endian_Reverse_32_Copy(void *Source, void *Destination);

/**
 * @brief   16位求和
 * @param   Address 起始地址
 * @param   Length 被加的数据的数量
 * @retval  uint16_t的求和结果
 * @note    接收**uint16_t 类型数组首地址**，把数组里Length个 16 位无符号整数全部累加，返回总和
 */
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);

/**
 * @brief 32位求和
 * @param Address 起始地址
 * @param Length 被加的数据的数量
 * @retval uint32_t 求和结果
 */
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);

/**
 * @brief sinc 函数实现
 * @param x 输入
 * @retval float 输出
 */
float Math_Sinc(float x);

#endif // __DRV_MATH_H
