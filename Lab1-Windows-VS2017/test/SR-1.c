#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 2000
#define ACK_TIMER 1000
#define MAX_SEQ 31
#define NR_BUFS ((MAX_SEQ + 1) / 2)
#define MAX_WINDOW_SIZE NR_BUFS

typedef unsigned char seq_nr; // 序列号类型

/* NAK 帧类型 */
#define FRAME_NAK 2

/* ACK 超时事件 */
#define ACK_TIMEOUT 5

struct FRAME
{
    unsigned char kind; /* 帧类型: FRAME_DATA, FRAME_ACK, 或 FRAME_NAK */
    seq_nr ack;
    seq_nr seq;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};

static int between(seq_nr a, seq_nr b, seq_nr c)
{
    /* 返回真，如果 a <= b < c (循环)；否则返回假 */
    if (((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a)))
        return 1;
    else
        return 0;
}

/* SR 协议状态变量 */
static seq_nr next_frame_to_send = 0;           // 发送窗口上界 + 1 (下一个待发送序号)
static seq_nr ack_expected = 0;                 // 发送窗口下界 (最早未确认帧)
static seq_nr frame_expected = 0;               // 接收窗口下界 (期望接收的帧)
static seq_nr too_far;                          // 接收窗口上界 + 1
static unsigned char out_buf[NR_BUFS][PKT_LEN]; // 发送方缓冲区
static unsigned char in_buf[NR_BUFS][PKT_LEN];  // 接收方缓冲区
static int nbuffered = 0;
static int phl_ready = 0;              // 物理层就绪标志
static unsigned char arrived[NR_BUFS]; // 接收缓冲区位图 (标记哪些槽已填充)
static int no_nak = 1;                 // 控制 NAK 发送频率 (1 表示可以发送 NAK)

/* 用于纯 SR ACK 处理的数组 (此实现未使用) */

static void put_frame(unsigned char *frame, int len)
{
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

/*将ACK和NAK合并，简化管理*/
static void send_frame_sr(unsigned char fk, seq_nr frame_nr, seq_nr frame_expected_by_receiver, unsigned char buffer[][PKT_LEN])
{
    struct FRAME s;
    s.kind = fk;
    s.seq = frame_nr;

    s.ack = (frame_expected_by_receiver + MAX_SEQ) % (MAX_SEQ + 1);

    if (fk == FRAME_DATA)
    {
        memcpy(s.data, buffer[frame_nr % NR_BUFS], PKT_LEN);
        dbg_frame("发送 DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);
        put_frame((unsigned char *)&s, 3 + PKT_LEN);
        start_timer(frame_nr, DATA_TIMER);
    }
    else if (fk == FRAME_ACK) // 发送 ACK 帧
    {
        dbg_frame("发送 ACK %d\n", s.ack);
        put_frame((unsigned char *)&s, 2);
        stop_ack_timer();
    }
    else if (fk == FRAME_NAK) // 发送 NAK 帧
    {
        /* NAK 帧在 ack 字段携带被 NAK 的帧序号 */
        s.ack = frame_nr; // NAK 帧序号 frame_nr
        no_nak = 0;       // 阻止连续发送 NAK
        dbg_frame("发送 NAK %d\n", s.ack);
        put_frame((unsigned char *)&s, 2);
    }
}

static void send_data_frame_sr(seq_nr frame_nr, seq_nr frame_expected_by_receiver)
{
    send_frame_sr(FRAME_DATA, frame_nr, frame_expected_by_receiver, out_buf);
}

static void send_ack_frame_sr(seq_nr ack_nr)
{
    send_frame_sr(FRAME_ACK, 0, ack_nr, NULL);
}

/* 发送 NAK 帧 (调用通用函数) */
static void send_nak_frame_sr(seq_nr nak_nr)
{
    send_frame_sr(FRAME_NAK, nak_nr, 0, NULL); // NAK 帧序号 nak_nr, ack 字段无意义
}

/* 启动/停止 ACK 计时器 (用于单独发送 ACK) */
static void start_ack_timer(void)
{
    start_timer(MAX_SEQ + 1, ACK_TIMER);
}

static void stop_ack_timer(void)
{
    stop_timer(MAX_SEQ + 1);
}

int main(int argc, char **argv)
{
    int event, arg;
    struct FRAME f;
    int len = 0;
    seq_nr i; // 循环变量

    protocol_init(argc, argv); // 协议初始化
    lprintf("Modified for SR, build: " __DATE__ "  " __TIME__ "\n");

    too_far = NR_BUFS;            // 初始化接收窗口上界
    for (i = 0; i < NR_BUFS; i++) // 初始化接收缓冲区标记
        arrived[i] = 0;

    enable_network_layer(); // 初始时允许网络层发送

    for (;;) // 主事件循环
    {
        event = wait_for_event(&arg);

        switch (event)
        {
        case NETWORK_LAYER_READY:
            if (nbuffered < NR_BUFS) // 检查发送窗口是否已满
            {
                get_packet(out_buf[next_frame_to_send % NR_BUFS]);             // 获取数据包存入发送缓冲区
                nbuffered++;                                                   // 增加发送缓冲区计数
                send_data_frame_sr(next_frame_to_send, frame_expected);        // 发送数据帧
                next_frame_to_send = (next_frame_to_send + 1) % (MAX_SEQ + 1); // 推进发送窗口上界
            }
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char *)&f, sizeof f); // 接收帧
            if (len < 5 || crc32((unsigned char *)&f, len) != 0)
            {
                dbg_event("**** 接收错误, CRC 校验失败\n");
                if (no_nak)
                    send_nak_frame_sr(frame_expected); // NAK 期望的帧
                break;                                 // 忽略损坏的帧
            }

            if (f.kind == FRAME_NAK)
            {
                dbg_frame("收到 NAK %d\n", f.ack);
                /* 检查被 NAK 的帧是否仍在发送窗口内 */
                if (between(ack_expected, f.ack, next_frame_to_send))
                {
                    send_data_frame_sr(f.ack, frame_expected); // 仅重传被 NAK 的帧
                }
            }
            else if (f.kind == FRAME_DATA) // 接收方收到 DATA
            {
                dbg_frame("收到 DATA %d %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                /* 检查帧是否在接收窗口内且未被接收过 */
                if (between(frame_expected, f.seq, too_far) && !arrived[f.seq % NR_BUFS])
                {

                    arrived[f.seq % NR_BUFS] = 1;                     // 标记缓冲区已填充
                    memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN); // 存入接收缓冲区

                    /* 发送 ACK (确认收到的帧 f.seq 或期望的 frame_expected) */

                    send_ack_frame_sr(frame_expected);

                    /* 向上层交付按序到达的帧 */
                    while (arrived[frame_expected % NR_BUFS]) // 只要期望的帧已到达
                    {
                        put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN);
                        no_nak = 1;                                            // 成功交付, 允许发送 NAK
                        arrived[frame_expected % NR_BUFS] = 0;                 // 清空缓冲区标记
                        frame_expected = (frame_expected + 1) % (MAX_SEQ + 1); // 滑动接收窗口下界
                        too_far = (too_far + 1) % (MAX_SEQ + 1);               // 滑动接收窗口上界
                        start_ack_timer();                                     // 重置 ACK 计时器 (若之后无数据到达, 则超时发送 ACK)
                    }
                }
                else if (!between(frame_expected, f.seq, too_far))
                {
                    /* 帧在窗口外, 需要重发对期望帧的 ACK */
                    send_ack_frame_sr(frame_expected);
                }
            }
            else if (f.kind == FRAME_ACK) // 发送方收到 ACK
            {
                dbg_frame("收到 ACK %d\n", f.ack);
                while (between(ack_expected, f.ack, next_frame_to_send)) // 确认号在窗口内
                {
                    nbuffered--;
                    stop_timer(ack_expected);
                    ack_expected = (ack_expected + 1) % (MAX_SEQ + 1);
                }
            }
            break;

        case DATA_TIMEOUT:
            dbg_event("---- 帧 %d 超时\n", arg);
            /* 仅重传超时的帧 */
            if (between(ack_expected, arg, next_frame_to_send))
            {
                send_data_frame_sr(arg, frame_expected);
            }
            break;

        case ACK_TIMEOUT: // ACK 计时器超时 (用于单独发送 ACK)
            dbg_event("---- ACK 超时\n");
            send_ack_frame_sr(frame_expected);
            break;
        }
        if (nbuffered < NR_BUFS)
            enable_network_layer();
        else
            disable_network_layer();
    }
}