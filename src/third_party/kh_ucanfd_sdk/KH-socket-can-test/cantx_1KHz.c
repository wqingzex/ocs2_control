#define _GNU_SOURCE  // 必需用于CPU亲和性函数
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <signal.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sched.h>        // 用于实时调度
#include <time.h>         // 用于高精度定时
#include <errno.h>        // 用于错误处理
#include <sys/mman.h>     // 用于 mlockall (内存锁定)
#include <stdint.h>       // 用于 uint32_t 类型

#define DEFAULT_INTERFACE "can0"
#define DEFAULT_CAN_ID 0x123
#define FRAME_PAYLOAD_SIZE 8
#define SEND_INTERVAL_NS 1000000L // 1ms in nanoseconds (1000000ns)
#define MIN_SLEEP_THRESHOLD_NS 500 // 最小睡眠阈值(500ns)
#define STATS_INTERVAL 100 // 每100帧输出一次统计信息

static int sockfd;
static volatile sig_atomic_t running = 1;
static int debug_mode = 0; // 调试模式开关

// 信号处理函数
static void signal_handler(int signo) {
    running = 0;
}

// 设置实时优先级
static int enable_realtime_scheduling(void) {
    struct sched_param param = {
        .sched_priority = sched_get_priority_max(SCHED_FIFO)
    };
    
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        perror("设置实时调度失败（请使用 root 运行）");
        return -1;
    }
    
    // 锁定内存防止页面错误
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("内存锁定失败");
    }
    
    // 设置CPU亲和性（绑定到CPU0）
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("设置CPU亲和性失败");
    }
    
    return 0;
}

int main(int argc, char *argv[]) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    const char *intf_name = DEFAULT_INTERFACE;
    struct timespec next;
    long min_jitter = 0, max_jitter = 0, total_jitter = 0;
    int frame_count = 0;
    
    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0) {
            debug_mode = 1;
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            intf_name = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0) {
            printf("用法: %s [-d] [-i interface]\n", argv[0]);
            printf("  -d   启用调试输出\n");
            printf("  -i   指定CAN接口 (默认: can0)\n");
            return 0;
        }
    }
    
    // 设置信号处理
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    
    // 设置实时优先级调度
    if (enable_realtime_scheduling() != 0) {
        fprintf(stderr, "以普通权限运行 (非实时模式)\n");
    } else {
        printf("启用实时调度优先级\n");
    }
    
    // 创建CAN套接字
    if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("创建CAN套接字失败");
        return 1;
    }
    
    // 绑定CAN接口
    strcpy(ifr.ifr_name, intf_name);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        perror("获取接口索引失败");
        close(sockfd);
        return 1;
    }
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("绑定接口失败");
        close(sockfd);
        return 1;
    }
    
    // 初始化CAN帧
    memset(&frame, 0, sizeof(frame));
    frame.can_id = DEFAULT_CAN_ID;
    frame.can_dlc = FRAME_PAYLOAD_SIZE;
    
    printf("在 %s 上以 %.1f kHz发送CAN帧... (Ctrl+C停止)\n", 
           intf_name, 1000000.0 / (SEND_INTERVAL_NS / 1000.0));
    
    // 获取初始时间
    if (clock_gettime(CLOCK_MONOTONIC, &next) != 0) {
        perror("获取时间失败");
        close(sockfd);
        return 1;
    }
    
    // 预热系统（跳过前几帧）
    for (int i = 0; i < 5; i++) {
        if (write(sockfd, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("预热帧发送失败");
        }
        usleep(1000);
    }
    
    struct timespec prev_time = next;
    
    while (running) {
        frame_count++;
        
        // 如果已到达发送时间点
        struct timespec current_time;
        if (clock_gettime(CLOCK_MONOTONIC, &current_time) != 0) {
            perror("获取当前时间失败");
            break;
        }
        
        // 计算抖动
        if (frame_count > 1) {
            long interval_ns = (current_time.tv_sec - prev_time.tv_sec) * 1000000000L;
            interval_ns += (current_time.tv_nsec - prev_time.tv_nsec);
            long jitter = labs(interval_ns - SEND_INTERVAL_NS);
            
            total_jitter += jitter;
            if (jitter < min_jitter || frame_count == 2) min_jitter = jitter;
            if (jitter > max_jitter) max_jitter = jitter;
        }
        prev_time = current_time;
        
        // 发送帧
        if (write(sockfd, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("发送帧失败");
            break;
        }
        
        // 计算下一帧的理想发送时间
        next.tv_nsec += SEND_INTERVAL_NS;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
        
        // 计算需要睡眠的时间
        struct timespec now;
        if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
            perror("获取结束时间失败");
            break;
        }
        
        // 计算时间差
        long delta_ns = (next.tv_sec - now.tv_sec) * 1000000000L;
        delta_ns += (next.tv_nsec - now.tv_nsec);
        
        // 每帧输出统计信息（仅在调试模式下）
        if (debug_mode) {
            printf("帧 %6d | 时间差: %6ld ns\n", frame_count, delta_ns);
        }
        
        // 只睡眠必要的时长
        if (delta_ns > MIN_SLEEP_THRESHOLD_NS) {
            struct timespec req = {
                .tv_sec = 0,
                .tv_nsec = delta_ns
            };
            nanosleep(&req, NULL);
        }
        // 如果delta_ns很小（<500ns），不睡眠以减少抖动
        
        // 更新帧计数器
        static uint32_t counter = 0;
        counter++;
        frame.data[0] = (counter >> 8) & 0xFF;
        frame.data[1] = counter & 0xFF;
        
        // 定期输出统计信息
        if (frame_count % STATS_INTERVAL == 0) {
            double avg_jitter = (double)total_jitter / STATS_INTERVAL;
            printf("帧 %6d | 抖动: min=%6ld ns | avg=%6.0f ns | max=%6ld ns\n",
                   frame_count, min_jitter, avg_jitter, max_jitter);
            
            // 重置统计
            min_jitter = 0;
            max_jitter = 0;
            total_jitter = 0;
        }
    }
    
    if (frame_count > 1) {
        double avg_jitter = (double)total_jitter / (frame_count - 1);
        printf("\n最终统计 | 总帧数: %d | 抖动: min=%6ld ns | avg=%6.0f ns | max=%6ld ns\n",
               frame_count, min_jitter, avg_jitter, max_jitter);
    }
    
    close(sockfd);
    return 0;
}