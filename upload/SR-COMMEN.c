#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"
#include <stdbool.h>

#define DATA_TIMER 2000
#define ACK_TIMER 1000
#define MAX_SEQ 7
#define NR_BUFS ((MAX_SEQ + 1) / 2) // 窗口大小 (必须是偶数)

struct FRAME
{
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};
static bool between(unsigned char a, unsigned char b, unsigned char c)
{
    return ((a <= b && b < c) || (c < a && a <= b) || (b < c && c < a));
}

static unsigned char nbuffered;
static unsigned char frame_expected = 0;
static int phl_ready = 0;
bool no_nak = true;
// static unsigned char oldest_fame = MAX_SEQ + 1; // 发送方最早未确认的帧序号
static unsigned char ack_expected;
static unsigned char next_frame_to_send;
static unsigned char too_far;

static unsigned char out_buf[NR_BUFS][PKT_LEN]; // 发送方缓冲区
static unsigned char in_buf[NR_BUFS][PKT_LEN];  // 接收方缓冲区
static bool arrived[NR_BUFS];                   // 接收方缓冲区位图 (标记哪些槽已填充)

static void put_frame(unsigned char *FRAME, int len)
{
    *(unsigned int *)(FRAME + len) = crc32(FRAME, len);
    send_frame(FRAME, len + 4);
    phl_ready = 0;
}

static void send_data_frame(unsigned char frame_nr)
{
    struct FRAME s;

    s.kind = FRAME_DATA;
    s.seq = frame_nr;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1); /* 0/1 序号空间大小为2 ，这里算出的就是下一帧*/
    memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN);

    dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);

    put_frame((unsigned char *)&s, 3 + PKT_LEN);
    start_timer(frame_nr % NR_BUFS, DATA_TIMER);
}

static void send_ack_frame(void)
{
    struct FRAME s;

    s.kind = FRAME_ACK;
    s.seq = next_frame_to_send;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    dbg_frame("Send ACK  %d\n", s.ack);

    put_frame((unsigned char *)&s, 2);
}

static void send_nak_frame(void)
{
    struct FRAME s;

    s.kind = FRAME_NAK;
    s.seq = 0;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    no_nak = false;

    dbg_frame("Send NAK with ACK %d\n", s.ack);

    put_frame((unsigned char *)&s, 2);
}

int main(int argc, char **argv)
{

    int i;
    struct FRAME f;

    int event, arg;
    int len = 0;

    disable_network_layer(); // 启用网络层
    ack_expected = 0;        // 发送方窗口下界 (最早未确认帧)
    next_frame_to_send = 0;  // 发送方窗口上界 + 1 (下一个待发送序号)
    frame_expected = 0;      // 接收方窗口下界 (期望接收的帧)
    too_far = NR_BUFS;       // 接收方窗口上界 + 1
    nbuffered = 0;           // 发送方缓冲区计数

    protocol_init(argc, argv);
    lprintf("Designed by Jiang Yanjun, build: " __DATE__ "  " __TIME__ "\n");

    for (i = 0; i < NR_BUFS; i++) // 初始化接收缓冲区标记
    {
        arrived[i] = false;
    }

    for (;;)
    {
        event = wait_for_event(&arg);

        switch (event)
        {
        case NETWORK_LAYER_READY:
            get_packet(&out_buf[next_frame_to_send % NR_BUFS]); // 获取数据包存入发送缓冲区
            nbuffered++;
            send_data_frame(next_frame_to_send % (MAX_SEQ + 1));           // 发送数据帧
            next_frame_to_send = (next_frame_to_send + 1) % (MAX_SEQ + 1); // 更新窗口上界, 继续循环

            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char *)&f, sizeof f);
            if (len < 5 || crc32((unsigned char *)&f, len) != 0)
            {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                if (no_nak)
                    send_nak_frame();
                break;
            }

            if (f.kind == FRAME_DATA)
            {
                dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short *)f.data);
                dbg_frame("frame_expected = %d, too_far = %d\n", frame_expected, too_far);
                if (f.seq != frame_expected && no_nak)
                {
                    send_nak_frame();
                    // break;
                }
                else
                {
                    start_ack_timer(ACK_TIMER); // 重置 ACK 计时器 (若之后无数据到达, 则超时发送 ACK)
                }

                if (between(frame_expected, f.seq, too_far) && !arrived[f.seq % NR_BUFS])
                {
                    arrived[f.seq % NR_BUFS] = true;                  // 标记缓冲区已填充
                    memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN); // 存入接收缓冲区
                    while (arrived[frame_expected % NR_BUFS])
                    {
                        put_packet(in_buf[frame_expected % NR_BUFS], len - 7); // 调整长度: 3 header + 4 CRC
                        no_nak = true;
                        arrived[frame_expected % NR_BUFS] = false;
                        frame_expected = (frame_expected + 1) % (MAX_SEQ + 1); // 滑动接收窗口下界
                        too_far = (too_far + 1) % (MAX_SEQ + 1);               // 滑动接收窗口上界
                        start_ack_timer(ACK_TIMER);                            // 重置 ACK 计时器 (若之后无数据到达, 则超时发送 ACK)
                    }
                }
            }
            if (f.kind == FRAME_NAK && between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send))
            {
                dbg_frame("Recv NAK with ACK %d\n", f.ack);
                send_data_frame((f.ack + 1) % (MAX_SEQ + 1));
            }
            while (between(ack_expected, f.ack, next_frame_to_send))
            {
                nbuffered--;
                stop_timer(ack_expected % NR_BUFS);
                ack_expected = (ack_expected + 1) % (MAX_SEQ + 1); // 向前滑动窗口下界
            }

            break;

        case DATA_TIMEOUT:
            dbg_event("---- DATA %d timeout\n", arg);
            if (between(ack_expected, arg, next_frame_to_send))
            {
                dbg_event("---- 重传超时的帧 %d\n", arg);
                send_data_frame(arg);
            }
            else
            {
                dbg_event("---- 超时的帧 %d 在窗口外 [%d, %d)\n", arg, ack_expected, next_frame_to_send);
            }
            break;
        case ACK_TIMEOUT:
            dbg_event("---- ACK %d timeout\n", arg);
            send_ack_frame(); // 重传 ACK 帧
            break;
        }

        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}
