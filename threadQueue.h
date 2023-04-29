#ifndef THREADQUEUE_H_
#define THREADQUEUE_H_

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#ifndef MULTIPLATFORMSLEEP_H_
#include "multiPlatformSleep.h"
#endif


#if __has_include("cameraGrabModule.h")    
#include <unistd.h>
#define LINUX
#else
#define NOMINMAX
#include <windows.h>
#endif

using namespace std::chrono_literals;

/**
* @tparam template for type universal queue
*/
template <typename T>

/**
* Default constructor for threadQueue class
*/
class threadQueue
{
public:

    /**
    * Function for pop from queue
    * @return queue item
    */
    T pop()
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (queue_.empty())
        {
            cond_.wait(mlock);            
        }
        auto item = queue_.front();
        queue_.pop();
        return item;
    }

    ///**
    //* Function for pop from queue with max wait time
    //* @return queue item
    //*/
    //T pop_timed()
    //{
    //    auto now = std::chrono::system_clock::now();
    //    std::unique_lock<std::mutex> mlock(mutex_);
    //    if (queue_.empty())
    //    {
    //        //if (cond_.wait_until(mlock, now + 100ms, []() {return i == 1; }))
    //        if(cond_.wait_until(mlock, now + 100ms, []() {return 1; }))
    //            std::cerr << "Queue " << queueName << " finished waiting."<< '\n';
    //        else
    //            std::cerr << "Queue " << queueName << " timed out. "<< '\n';
    //        //cond_.wait(mlock);
    //    }
    //    auto item = queue_.front();
    //    queue_.pop();
    //    return item;
    //}

    /**
    * Function for pop from queue
    * @param for poped item from queue
    */
    void pop(T& item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (queue_.empty())
        {            
            cond_.wait(mlock);
        }
        item = queue_.front();
        queue_.pop();
    }

    /**
    * Function for push item to queue
    * @param item to push into queue
    * @return int 0 if item was pushed succesfully or 1 if queue is full
    */
    int push(T& item)
    {
        while (queue_.size() > 20)
        {
            std::cout << "Queue " << queueName << " is full!" << std::endl;
            sleep(10);
            TTL--;
            if (TTL < 1)
                return 1;                
        }
        TTL = 50;
        std::unique_lock<std::mutex> mlock(mutex_);
        queue_.push(item);
        mlock.unlock();
        cond_.notify_one();
        return 0;
    }
    /**
    * Function for push item to queue
    * @param item to push into queue
    * @return int 0 if item was pushed succesfully or 1 if queue is full
    */
    int push(T&& item)
    {
        while (queue_.size() > 20)
        {
            std::cout << "Queue " << queueName << " is full!" << std::endl;
            sleep(10);
            TTL--;
            if (TTL < 1)
                return 1;
        }
        TTL = 50;
        std::unique_lock<std::mutex> mlock(mutex_);
        queue_.push(std::move(item));
        mlock.unlock();
        cond_.notify_one();
        return 0;
    }

    /**
    * Function for get size of queue
    * @return int is number of items in queue
    */
    int size()
    {
        return queue_.size();
    }

    std::string queueName; /*!< String for name of queue */

private:
    std::queue<T> queue_; /*!< Queue */
    std::mutex mutex_; /*!< mutex_ for thread safe */
    std::condition_variable cond_; /*!< cond_ for thread safe */
    int TTL = 50; /*!< Maximum number of try push into queue before return 1 as queue is full */
    //std::atomic<int> i{ 0 };
};

#endif