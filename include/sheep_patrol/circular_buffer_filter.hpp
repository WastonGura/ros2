#ifndef SHEEP_PATROL_CIRCULAR_BUFFER_FILTER_HPP_
#define SHEEP_PATROL_CIRCULAR_BUFFER_FILTER_HPP_

#include <cstddef>

namespace sheep_patrol
{

/**
 * @brief 基于循环缓冲区（Ring / Circular Buffer）的滑动平均滤波算法类
 * 
 * 数据结构特点与教学考点：
 * 1. 内部采用固定大小的数组实现环形缓冲，内存开销恒定，无动态内存重分配开销。
 * 2. 插入新采样并覆盖老样本的时间复杂度为 O(1)。
 * 3. 维护增量局部和 sum_，更新滑动平均值的时间复杂度恒为 O(1)。
 */
template <size_t Capacity = 15>
class CircularBufferFilter
{
public:
    CircularBufferFilter()
    : head_(0), count_(0), sum_(0.0)
    {
        for (size_t i = 0; i < Capacity; ++i) {
            buffer_[i] = 0.0;
        }
    }

    /**
     * @brief 压入新采集的原始数据，并返回最新的滑动平均滤波值
     * @param val 传入的带噪声温度采样值
     * @return double 滤波后的平滑温度
     */
    double update(double val)
    {
        if (count_ < Capacity) {
            buffer_[head_] = val;
            sum_ += val;
            count_++;
        } else {
            sum_ -= buffer_[head_];
            buffer_[head_] = val;
            sum_ += val;
        }
        head_ = (head_ + 1) % Capacity;
        return (count_ > 0) ? (sum_ / static_cast<double>(count_)) : val;
    }

    /**
     * @brief 重置环形缓冲区
     */
    void reset()
    {
        head_ = 0;
        count_ = 0;
        sum_ = 0.0;
        for (size_t i = 0; i < Capacity; ++i) {
            buffer_[i] = 0.0;
        }
    }

    /**
     * @brief 获取当前有效样本数
     */
    size_t size() const { return count_; }

    /**
     * @brief 获取缓冲区容量上限
     */
    size_t capacity() const { return Capacity; }

    /**
     * @brief 缓冲区是否已被充满
     */
    bool full() const { return count_ == Capacity; }

private:
    double buffer_[Capacity];
    size_t head_;
    size_t count_;
    double sum_;
};

}  // namespace sheep_patrol

#endif  // SHEEP_PATROL_CIRCULAR_BUFFER_FILTER_HPP_
