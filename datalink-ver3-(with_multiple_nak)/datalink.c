#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <windows.h>

#include "protocol.h"

#define DATA_TIMER 8000 
#define ACK_TIMER 260   
#define RETRX_TIMER 100
#define MAX_SEQ 127
#define NR_BUFS ((MAX_SEQ + 1) / 2) 
#define MAX_WINDOW_SIZE NR_BUFS

typedef unsigned char seq_nr;

/* FRAME kind */
#define FRAME_DATA 0
#define FRAME_ACK 1
#define FRAME_NAK 2
#define FRAME_HALF_ACK 3

#define inc(k)       \
    if (k < MAX_SEQ) \
        k++;         \
    else             \
        k = 0

struct FRAME
{
    unsigned char kind;
    seq_nr ack;
    seq_nr seq;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};

static bool between(seq_nr a, seq_nr b, seq_nr c) 
{
    return ((a <= b && b < c) || (c < a && a <= b) || (b < c && c < a));
}


static seq_nr next_frame_to_send = 0; 
static seq_nr ack_expected = 0;       
static seq_nr frame_expected = 0;     
static seq_nr too_far;                

static unsigned char out_buf[NR_BUFS][PKT_LEN]; 
static unsigned char in_buf[NR_BUFS][PKT_LEN];  
static bool arrived[NR_BUFS];                   

static int nbuffered = 0;    
static int phl_ready = 1;    
static bool no_nak[NR_BUFS]; 
static bool no_retransmission = true;
static seq_nr retransmission_seq;

//static void show_received_buffer()
//{
//    printf("\033[33m                                             ");
//    for (int i = 0; i < 16; i++)
//    {
//        printf("|");
//        if (arrived[i])
//            printf("%2d", i % NR_BUFS);
//        else
//            printf("  ");
//    }
//    printf("|\033[0m\n");
//}

static void put_frame(unsigned char *frame, int len) 
{
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

static void send_data_frame(seq_nr frame_nr)
{
    struct FRAME s;
    s.kind = FRAME_DATA;
    s.seq = frame_nr;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);   
    memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN); 
    dbg_frame("\033[32mSend DATA %d %d, ID %d [%d|%d) phl_sq:%d\033[0m\n", s.seq, s.ack, *(short*)s.data, ack_expected, next_frame_to_send, phl_sq_len());
    put_frame((unsigned char *)&s, 3 + PKT_LEN);
    start_timer(frame_nr % NR_BUFS, DATA_TIMER); 
    stop_ack_timer();
}

static void send_ack_frame(void)
{
    struct FRAME s;
    s.kind = FRAME_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
    s.seq = next_frame_to_send;

    dbg_frame("Send ACK %d \n", s.ack);
    put_frame((unsigned char *)&s, 2);
    stop_ack_timer();
}


static void send_nak_frame(seq_nr nak_send)
{
    struct FRAME s;
    s.kind = FRAME_NAK;

    s.ack = (nak_send + MAX_SEQ) % (MAX_SEQ + 1);


    no_nak[nak_send % NR_BUFS] = false; 

    dbg_frame("Send NAK (ack=%d, expect%d)\n", s.ack, s.ack + 1);
    put_frame((unsigned char *)&s, 2);
    stop_ack_timer();
}

static void whether_send_half_ack()
{
    struct FRAME s;
    s.kind = FRAME_HALF_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    seq_nr num = 0;
    for (seq_nr i = 0; i < NR_BUFS; i++)
    {
        if (arrived[i])
        {
            s.data[num] = (i < too_far && i >= frame_expected) ? i : (i + NR_BUFS) % (MAX_SEQ + 1); // ����/����ʱֻҪ����һ���ڴ����о��У�
            num = num + 1;
        }
    }

    s.seq = num; 

    if (num > 0)
    {
        put_frame((unsigned char *)&s, 3 + num);
        dbg_frame("Send half ACK,???????%d  ", num);
        //for (seq_nr i = 0; i < num; i++)
        //{
        //    printf("%d ", s.data[i]);
        //}
        dbg_frame("\n");
    }
}

int main(int argc, char **argv)
{
    SetConsoleOutputCP(65001);
    int event;
    seq_nr arg; 
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv);

    too_far = NR_BUFS;
    nbuffered = 0;
    phl_ready = 0;
    memset(no_nak, true, sizeof(no_nak));
    memset(arrived, 0, sizeof(arrived));

    disable_network_layer();

    for (;;)
    {
        event = wait_for_event((int *)&arg); 

        switch (event)
        {
        case NETWORK_LAYER_READY:
            if (nbuffered < NR_BUFS)
            {
                get_packet(out_buf[next_frame_to_send % NR_BUFS]);
                nbuffered++;
                send_data_frame(next_frame_to_send);
                inc(next_frame_to_send);
            }
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char *)&f, sizeof f);

            if (len < 5 || crc32((unsigned char *)&f, len) != 0)
            {
                send_nak_frame(f.seq); 
                switch (f.kind) 
                {
                case FRAME_DATA:
                    dbg_event("**** CRC错误 %d号帧错误\n", f.seq);
                    break;
                case FRAME_ACK:
                    dbg_event("**** CRC错误 ACK %d错误\n", f.ack);
                    break;
                case FRAME_NAK:
                    dbg_event("**** CRC错误 NAK %d %d错误\n", f.ack, f.ack + 1);
                    break;
                }

                //if (no_nak[frame_expected % NR_BUFS] && f.kind != FRAME_HALF_ACK)
                //{
                //    whether_send_half_ack();
                //}
                break;
            }

            if (f.kind == FRAME_DATA)
            {
                dbg_frame("\033[33mrecive DATA %d %d, ID %d             [%d,%d)\033[0m\n", f.seq, f.ack, *(short *)f.data, frame_expected, too_far);

                if (f.seq != frame_expected && no_nak[frame_expected % NR_BUFS])
                {
                    send_nak_frame(frame_expected);
                    //whether_send_half_ack();
                }
                else
                {
                    start_ack_timer(ACK_TIMER);
                }

                if (between(frame_expected, f.seq, too_far)) 
                {
                    if (!arrived[f.seq % NR_BUFS]) 
                    {

                        arrived[f.seq % NR_BUFS] = true;
                        memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);

                        while (arrived[frame_expected % NR_BUFS])
                        {
                            put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN);

                            no_nak[frame_expected % NR_BUFS] = true;
                            arrived[frame_expected % NR_BUFS] = false;
                            inc(frame_expected);
                            inc(too_far);
                            start_ack_timer(ACK_TIMER);
                        }
                    }
                    else
                    {
                        dbg_frame("\033[31mDATA %d already received\033[0m\n", f.seq);
                        start_ack_timer(ACK_TIMER);
                    }
                }
                else
                {
                    dbg_frame("\n"); 
                    dbg_frame("\033[31mDATA %d out of window [%d, %d)\033[0m\n", f.seq, frame_expected, too_far);
                    start_ack_timer(ACK_TIMER);
                }
            }

            if (f.kind == FRAME_NAK)
            {
                seq_nr missing_seq = (f.ack + 1) % (MAX_SEQ + 1); 
                dbg_frame("Recv NAK (ack=%d), 推断丢失 %d\n", f.ack, missing_seq);

                if (between(ack_expected, missing_seq, next_frame_to_send))
                {
                    dbg_frame("resend %d (receive NAK)\n", missing_seq);
                    send_data_frame(missing_seq);
                }
                else
                {
                    dbg_frame("推断丢失的帧 %d 不在窗口 [%d, %d) 内, 忽略 NAK\n", missing_seq, ack_expected, next_frame_to_send);
                }
                 break; 
            }

            if (f.kind == FRAME_ACK)
            {
                dbg_frame("Recv ACK %d\n", f.ack);
            }

            if (f.kind == FRAME_HALF_ACK)
            {
                dbg_frame("Recv half ACK, 确认个数为%d  ", f.seq);
                dbg_event("停止计时");
                for (seq_nr i = 0; i < f.seq; i++)
                {
                    //printf("%d ", f.data[i]);
                    stop_timer((unsigned int)(f.data[i] % NR_BUFS));
                }
                dbg_frame("\n");
            }

            while (between(ack_expected, f.ack, next_frame_to_send))
            {
                if (no_retransmission == false && retransmission_seq == ack_expected)
                    no_retransmission = true;

                nbuffered--;
                stop_timer(ack_expected % NR_BUFS);
                dbg_event("停记%d ", ack_expected % NR_BUFS);
                inc(ack_expected);
            }
            dbg_frame("\n");
            break;

        case DATA_TIMEOUT:
            if (between(ack_expected, arg, next_frame_to_send))
            {
                if (no_retransmission)
                {
                    dbg_event("---- DATA %d 超时 (来自 wait_for_event) 重传超时的帧\n", arg);
                    send_data_frame(arg);
                    retransmission_seq = arg;
                    no_retransmission = false;
                }
                else if (between(ack_expected, arg, retransmission_seq + 1)) 
                {
                    dbg_event("---- DATA %d 超时 (来自 wait_for_event) 重传超时的帧\n", arg);
                    send_data_frame(arg);
                    retransmission_seq = arg;
                    no_retransmission = false;
                }
                else 
                {
                    dbg_event("---- DATA %d 超时，延时300ms\n", arg);
                    start_timer(arg, RETRX_TIMER);
                }
            }
            else
            {
                if (no_retransmission) 
                {
                    dbg_event("---- DATA %d 超时，重传\n", (arg + NR_BUFS) % (MAX_SEQ + 1));
                    send_data_frame((arg + NR_BUFS) % (MAX_SEQ + 1));
                    retransmission_seq = (arg + NR_BUFS) % (MAX_SEQ + 1);
                    no_retransmission = false;
                }
                else if (between(ack_expected, (arg + NR_BUFS) % (MAX_SEQ + 1), retransmission_seq + 1)) 
                {
                    dbg_event("---- DATA %d 超时，重传\n", (arg + NR_BUFS) % (MAX_SEQ + 1));
                    send_data_frame((arg + NR_BUFS) % (MAX_SEQ + 1));
                    retransmission_seq = (arg + NR_BUFS) % (MAX_SEQ + 1);
                    no_retransmission = false;
                }
                else 
                {
                    dbg_event("---- DATA %d 超时，延时300ms\n", (arg + NR_BUFS) % (MAX_SEQ + 1));
                    start_timer(arg, RETRX_TIMER);
                }
            }
            break;

        case ACK_TIMEOUT:
            dbg_event("---- ACK???\n");
            send_ack_frame();
            break;
        }

        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}