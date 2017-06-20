#include "wave/matching/multi_matcher.hpp"

namespace wave {

template<class T, class R>
void MultiMatcher<T, R>::initPool(std::string path) {
    this->config = path;
    for (int i = 0; i < this->n_thread; i++) {
        if (this->config.empty()) {
            this->matchers = T(R());
        } else {
            this->matchers = T(R(this->config));
        }
        this->pool.push_back(std::thread(&(this->spin), this, i));
    }
}

template<class T, class R>
void MultiMatcher<T, R>::spin(int threadid) {
    std::unique_lock<std::mutex> lock(this->ip_mutex);
    std::unique_lock<std::mutex> lockop(this->op_mutex);
    while (true) {
        lock.lock();
        if (this->input.size() > 0) {
            auto val = this->input.front();
            this->input.pop();
            lock.unlock();
            this->matchers.at(threadid).setRef(std::get<1>(val));
            this->matchers.at(threadid).setTarget(std::get<2>(val));
            this->matchers.at(threadid).match();
            this->matchers.at(threadid).estimateInfo();
            lockop.lock();
            this->output.emplace(std::get<0>(val),
                                 this->matchers.at(threadid).getResult(),
                                 this->matchers.at(threadid).getInfo());
            lockop.unlock();
        } else {
            lock.unlock();
        }
    }
}

template<class T, class R>
void MultiMatcher<T, R>::insert(const int &id,
                          const PCLPointCloud &src,
                          const PCLPointCloud &target) {
    std::unique_lock<std::mutex> lock(this->ip_mutex);
    while(true) {
        lock.lock();
        if (this->input.size() < this->queue_size) {
            this->input.emplace(id, src, target);
            lock.unlock();
            break;
        } else {
            lock.unlock();
        }
    }
}
}
