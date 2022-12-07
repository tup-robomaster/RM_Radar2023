#ifndef __SHAREDQUEUE_H
#define __SHAREDQUEUE_H
 
#include <queue>
#include <mutex>
#include <condition_variable>
 
template <typename T>
class SharedQueue
{
public:
    SharedQueue();
    ~SharedQueue();
 
    T &front();
    void pop();
 
    void push(const T &item);
    void push(T &&item);
 
    size_t size();
    bool empty();
 
private:
    std::deque<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
};
 
template <typename T>
SharedQueue<T>::SharedQueue(){};
 
template <typename T>
SharedQueue<T>::~SharedQueue() {}
 
template <typename T>
bool SharedQueue<T>::empty()
{
    return size() ? false : true;
}
 
template <typename T>
T &SharedQueue<T>::front()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    return queue_.front();
}
 
template <typename T>
void SharedQueue<T>::pop()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    queue_.pop_front();
}
 
template <typename T>
void SharedQueue<T>::push(const T &item)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push_back(item);
    mlock.unlock();     // unlock before notificiation to minimize mutex con
    cond_.notify_one(); // notify one waiting thread
}
 
template <typename T>
void SharedQueue<T>::push(T &&item)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push_back(std::move(item));
    mlock.unlock();     // unlock before notificiation to minimize mutex con
    cond_.notify_one(); // notify one waiting thread
}
 
template <typename T>
size_t SharedQueue<T>::size()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    size_t size = queue_.size();
    mlock.unlock();
    return size;
}

#endif