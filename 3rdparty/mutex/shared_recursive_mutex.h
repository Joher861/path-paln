#ifndef MUTEX_SHARED_RECURSIVE_MUTEX_H
#define MUTEX_SHARED_RECURSIVE_MUTEX_H

#include <shared_mutex>
#include <thread>
#include <atomic>
#include <map>

class shared_recursive_mutex : public std::shared_mutex
{
  public:
    void lock(void) {
        std::thread::id this_id = std::this_thread::get_id();
        if(owner == this_id) {
            // recursive locking
            count++;
        }
        else {
            // normal locking
            std::shared_mutex::lock();
            owner = this_id;
            count = 1;
        }
    }
    void unlock(void) {
        if(count > 1) {
            // recursive unlocking
            count--;
        }
        else {
            // normal unlocking
            owner = std::thread::id();
            count = 0;
            std::shared_mutex::unlock();
        }
    }

private:
    std::atomic<std::thread::id> owner;
    int count;
};

// class shared_recursive_mutex
// {
// public:
//     void lock(void)
//     {
//         std::thread::id this_id = std::this_thread::get_id();
//         if (owner == this_id)
//         {
//             // recursive locking
//             ++count;
//         }
//         else
//         {
//             // normal locking
//             m.lock();
//             owner = this_id;
//             count = 1;
//         }
//     }
//     void unlock(void)
//     {
//         if (count > 1)
//         {
//             // recursive unlocking
//             count--;
//         }
//         else
//         {
//             // normal unlocking
//             owner = std::thread::id();
//             count = 0;
//             m.unlock();
//         }
//     }
//     void lock_shared()
//     {
//         std::thread::id this_id = std::this_thread::get_id();
//         if (shared_counts->count(this_id))
//         {
//             ++(shared_counts.get_locked()[this_id]);
//         }
//         else
//         {
//             m.lock_shared();
//             shared_counts.get_locked()[this_id] = 1;
//         }
//     }
//     void unlock_shared()
//     {
//         std::thread::id this_id = std::this_thread::get_id();
//         auto it = shared_counts->find(this_id);
//         if (it->second > 1)
//         {
//             --(it->second);
//         }
//         else
//         {
//             shared_counts->erase(it);
//             m.unlock_shared();
//         }
//     }

// private:
//     std::shared_mutex m;
//     std::atomic<std::thread::id> owner;
//     std::atomic<std::size_t> count;
//     std::map<std::thread::id, std::size_t> shared_counts;
//     // mutex_guarded<std::map<std::thread::id, std::size_t>> shared_counts;
// };

#endif // MUTEX_SHARED_RECURSIVE_MUTEX_H