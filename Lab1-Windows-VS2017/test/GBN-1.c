#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 2000
#define MAX_SEQ 31
#define MAX_WINDOW_SIZE MAX_SEQ

typedef unsigned char seq_nr;

struct FRAME
{
    unsigned char kind;
    seq_nr ack;
    seq_nr seq;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};

/* 判断一个序号是否在窗口内 */
static int between(seq_nr a, seq_nr b, seq_nr c)
{
    /* 正常情况：a <= b < c */
    if (((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a)))
        return 1;
    else
        return 0;
}

/* Global variables for GBN state */
static seq_nr next_frame_to_send = 0;              /* 序号范围: [0, MAX_SEQ] */
static seq_nr ack_expected = 0;                    /* 发送窗口下界 */
static seq_nr frame_expected = 0;                  /* 接收方期望收到的下一个帧序号 */
static unsigned char buffer[MAX_SEQ + 1][PKT_LEN]; /* 改为存储窗口数量的数据包 */
static int nbuffered = 0;                          /* 窗口内帧数量 */
static int phl_ready = 0;                          /* Physical layer readiness (simplified) */
// Note: GBN typically uses a single timer for the oldest unacked frame (ack_expected).
// We'll adapt the existing timer functions conceptually.

static void put_frame(unsigned char *frame, int len)
{
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

static void send_data_frame(seq_nr frame_nr, seq_nr frame_expected_by_receiver)
{
    struct FRAME s;

    s.kind = FRAME_DATA;
    s.seq = frame_nr;
    /* 计算捎带确认号，表示接收方期望的下一个帧序号 */
    s.ack = (frame_expected_by_receiver + MAX_SEQ) % (MAX_SEQ + 1);
    /*当frame_expected_by_receiver为0时: (0 + MAX_SEQ) % (MAX_SEQ + 1) = MAX_SEQ*/
    /*对于其他序号n(n>0): (n + MAX_SEQ) % (MAX_SEQ + 1) = n - 1*/
    memcpy(s.data, buffer[frame_nr], PKT_LEN);

    dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);

    put_frame((unsigned char *)&s, 3 + PKT_LEN);
    start_timer(frame_nr, DATA_TIMER);
}

static void send_ack_frame(void)
{
    struct FRAME s;

    s.kind = FRAME_ACK;
    /* 已正确接收所有序号小于frame_expected的帧 */
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    dbg_frame("Send ACK %d\n", s.ack);

    put_frame((unsigned char *)&s, 2);
}

static void start_ack_timer(void)
{
    start_timer(ack_expected, DATA_TIMER);
}

/* Stop the single timer */
static void stop_ack_timer(void)
{
    stop_timer(ack_expected);
}

int main(int argc, char **argv)
{
    int event, arg;
    struct FRAME f;
    int len = 0;
    seq_nr i;

    protocol_init(argc, argv);
    lprintf("Modified for GBN, build: " __DATE__ "  " __TIME__
            "\n");

    enable_network_layer();

    for (;;)
    {
        event = wait_for_event(&arg);

        switch (event)
        {
        case NETWORK_LAYER_READY:
            if (nbuffered < MAX_WINDOW_SIZE)
            {                                                                  /* 先检查窗口是否已满 */
                get_packet(buffer[next_frame_to_send]);                        /* 获取数据包并存入窗口中的特定位置 */
                nbuffered++;                                                   /* Increment buffer count */
                send_data_frame(next_frame_to_send, frame_expected);           /* 序号和接收方期望序号 */
                next_frame_to_send = (next_frame_to_send + 1) % (MAX_SEQ + 1); /* 更新窗口上界, 继续循环 */
            }
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char *)&f, sizeof f);

            if (len < 5 || crc32((unsigned char *)&f, len) != 0)
            {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                break;
            }

            /* 接收方累计确认 */
            if (f.kind == FRAME_DATA)
            {
                dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                /* 累计确认机制 */
                if (f.seq == frame_expected)
                {
                    /* Pass data to network layer */
                    put_packet(f.data, len - 7); /* Adjust length: 3 header + 4 CRC */

                    frame_expected = (frame_expected + 1) % (MAX_SEQ + 1);
                }
                /*无论是否接受当前帧，都会调用*/
                send_ack_frame();
            }

            /* 发送方处理 */
            /* 循环处理一个ACK可能确认的多个帧 */
            while (between(ack_expected, f.ack, next_frame_to_send))
            {
                /* Frame ack_expected has been acknowledged */
                nbuffered--;
                stop_timer(ack_expected);
                ack_expected = (ack_expected + 1) % (MAX_SEQ + 1); /* 向前滑动窗口下界 */
            }
            break;

        case DATA_TIMEOUT:
            /* 检查超时帧是否是窗口最早未确认帧 */
            if (arg == ack_expected)
            {
                dbg_event("---- Timeout for frame %d. Resending window.\n", ack_expected);
                next_frame_to_send = ack_expected;
                /*使用循环重传整个窗口中所有未确认的帧*/
                for (i = 0; i < nbuffered; i++)
                {
                    send_data_frame(next_frame_to_send, frame_expected);
                    next_frame_to_send = (next_frame_to_send + 1) % (MAX_SEQ + 1);
                }
            }
            else
            {
                dbg_event("---- Timeout for frame %d (ignored, waiting for ack_expected %d timeout)\n", arg, ack_expected);
            }
            break;
        }

        /* 流量控制 */
        if (nbuffered < MAX_WINDOW_SIZE)
            enable_network_layer();
        else
            disable_network_layer();
    }
}