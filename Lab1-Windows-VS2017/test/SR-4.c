#define MAX_SEQ 7
#define NR_BUFS ((MAX_SEQ + 1) / 2) // 窗口大小 (必须是偶数)

typedef enum
{
    frame_arrival,
    cksum_err,
    timeout,
    network_layer_ready,
    ack_timeout
} event_type; // 事件类型

#include "protocol.h"

#include "datalink.h"

boolean no_nak = true;
seq_nr oldest_fame = MAX_SEQ + 1; // 发送方最早未确认的帧序号

static boolean between(seq_nr a, seq_nr b, seq_nr c) // 判断b是否在a和c之间
{
    return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

static void send_frame(frame_kind fk, seq_nr frame_nr, seq_nr frame_expected, packet buffer[])
{
    frame s;
    s.kind = fk;
    if (fk == data)
    {
        s.info = buffer[frame_nr % NR_BUFS]; // 发送数据帧时，携带数据
    }
    s.seq = frame_nr;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1); // 发送帧时，携带确认号
    if (fk == nak)
    {
        no_nak = false; // 发送NAK时，禁止连续发送NAK
    }
    dbg_frame("发送 %s %d %d, ID %d\n", fk == data ? "DATA" : (fk == ack ? "ACK" : "NAK"), s.seq, s.ack, *(short *)s.info);
    to_physical_layer(&s);
    if (fk == data)
    {
        start_timer(frame_nr % NR_BUFS);
    }
    stop_ack_timer(); // 停止ACK计时器
}

int main()
{
    seq_nr ack_expected;
    seq_nr next_frame_to_send;
    seq_nr frame_expected;
    seq_nr too_far;
    int i;
    frame r;
    packet out_buf[NR_BUFS];  // 发送方缓冲区
    packet in_buf[NR_BUFS];   // 接收方缓冲区
    boolean arrived[NR_BUFS]; // 接收方缓冲区位图 (标记哪些槽已填充)
    seq_nr nbuffered;
    event_type event;

    enable_network_layer(); // 启用网络层
    ack_expected = 0;       // 发送方窗口下界 (最早未确认帧)
    next_frame_to_send = 0; // 发送方窗口上界 + 1 (下一个待发送序号)
    frame_expected = 0;     // 接收方窗口下界 (期望接收的帧)
    too_far = NR_BUFS;      // 接收方窗口上界 + 1
    nbuffered = 0;          // 发送方缓冲区计数

    for (i = 0; i < NR_BUFS; i++) // 初始化接收缓冲区标记
    {
        arrived[i] = false;
    }

    for (;;)
    {
        wait_for_event(&event); // 等待事件发生
        switch (event)
        {
        case network_layer_ready:
            nbuffered++;
            from_network_layer(&out_buf[next_frame_to_send % NR_BUFS]);    // 获取数据包存入发送缓冲区
            send_frame(data, next_frame_to_send, frame_expected, out_buf); // 发送数据帧
            inc(next_frame_to_send);                                       // 推进发送窗口上界
            break;
        case frame_arrival:
            from_physical_layer(&r); // 接收帧
            if (r.kind == data)
            {
                if ((r.seq != frame_expected) && no_nak) // 如果收到的帧不在接收窗口内，发送NAK
                {
                    send_frame(nak, r.seq, frame_expected, out_buf); // 发送NAK帧
                }
                else
                {
                    start_ack_timer(); // 启动ACK计时器
                }
                if (between(fram_expected, r.seq, too_far) && (arrived[r.seq % NR_BUFS] == false)) // 如果收到的帧在接收窗口内且未被接收过
                {
                    arrived[r.seq % NR_BUFS] = true;          // 标记缓冲区已填充
                    in_buf[r.seq % NR_BUFS] = r.info;         // 存入接收缓冲区
                    while (arrived[frame_expected % NR_BUFS]) // 只要期望的帧已到达
                    {
                        to_network_layer(&in_buf[frame_expected % NR_BUFS]); // 向上层交付按序到达的帧
                        no_nak = true;                                       // 成功交付, 允许发送 NAK
                        arrived[frame_expected % NR_BUFS] = false;           // 清空缓冲区标记
                        inc(frame_expected);                                 // 滑动接收窗口下界
                        inc(too_far);                                        // 滑动接收窗口上界
                        start_ack_timer();                                   // 重置 ACK 计时器 (若之后无数据到达, 则超时发送 ACK)
                    }
                }
                // else if(!between(frame_expected,r.seq,too_far)) // 如果帧在窗口外，重发对期望帧的ACK
                // {
                //     send_frame(ack, frame_expected, frame_expected, out_buf); // 发送ACK帧
                // }
            }
            if ((r.kind == nak) && between(ack_expected, (r.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) // 如果收到的帧是NAK，且在发送窗口内
            {
                send_frame(data, (r.ack + 1) % (MAX_SEQ + 1), frame_expected, out_buf); // 发送数据帧
            }
            while (between(ack_expected, r.ack, next_frame_to_send)) // 确认号在窗口内
            {
                nbuffered--;                        // 减少发送方缓冲区计数
                stop_timer(ack_expected % NR_BUFS); // 停止计时器
                inc(ack_expected);                  // 滑动发送窗口下界
            }
            break;
        case cksum_err:
            // 处理校验和错误
            if (no_nak)
            {
                send_frame(nak, 0, frame_expected, out_buf); // 发送NAK帧
            }
            break;
        case timeout:
            send_frame(data, oldest_frame, frame_expected, out_buf); // 重传超时的帧
            break;
        case ack_timeout:
            send_frame(ack, 0, frame_expected, out_buf); // 发送ACK帧
        }
        if (nbuffered < NR_BUFS)
        {
            enable_network_layer(); // 启用网络层
        }
        else
        {
            disable_network_layer(); // 禁用网络层
        }
    }
}