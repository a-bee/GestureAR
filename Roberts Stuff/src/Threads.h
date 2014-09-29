#pragma once

#include <functional>

#if defined(_WIN32)
// Use native Windows threads:
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    ifndef WIN32_LEAN_AND_MEAN
#        define WIN32_LEAN_AND_MEAN
#    endif
#    include <Windows.h>
#else
// Use pthreads on other platforms:
#    include <pthread.h>
#endif

class Mutex
{
    friend class SimpleLock;
public:
	Mutex();
	~Mutex();

private:
#ifdef _WIN32
	HANDLE hMutex_;
#else
	pthread_mutex_t posixMutex_;
#endif
};

class SimpleLock
{
public:
    explicit SimpleLock(Mutex& mutex);
    ~SimpleLock();
    
private:
    Mutex& mutex_;
};

class Thread
{
public:
    explicit Thread(std::function<void (void)> f);
    ~Thread();
    void join();
    static void sleep(short millis);

private:
    std::function<void (void)> f_;
#ifdef _WIN32
    static DWORD wrapperFunction(LPVOID lpParam);

    HANDLE hThread_;
#else
    static void* wrapperFunction(void* lpParam);

    pthread_t posixThreadId_;
#endif
};

