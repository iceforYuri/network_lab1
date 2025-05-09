#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 2000 // Data֡��ʱ
#define ACK_TIMER 260   // Ack֡��ʱ
#define MAX_SEQ 127
#define NR_BUFS ((MAX_SEQ + 1) / 2) // ���ڴ�С (������ż��)
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

static bool between(seq_nr a, seq_nr b, seq_nr c) // �ж�����Ƿ��ڴ�����
{
    return ((a <= b && b < c) || (c < a && a <= b) || (b < c && c < a));
}

// --- ȫ�ֱ��� ---
static seq_nr next_frame_to_send = 0; // ���ͷ���һ��Ҫ���͵�֡���
static seq_nr ack_expected = 0;       // ���ͷ���һ��Ҫȷ�ϵ�֡���
static seq_nr frame_expected = 0;     // ���շ���һ��Ҫ���յ�֡���
static seq_nr too_far;                // ���շ���һ��Ҫ���յ�֡��� (�����Ͻ�)

static unsigned char out_buf[NR_BUFS][PKT_LEN]; // ���ͷ�������
static unsigned char in_buf[NR_BUFS][PKT_LEN];  // ���շ�������
static bool arrived[NR_BUFS];                   // ���շ�������λͼ (�����Щ�������)

static int nbuffered = 0;  // ���ͷ����������Ѵ�ŵ�֡��
static int phl_ready = 1;  // ������Ƿ�׼���ý�������
static bool no_nak = true; // �Ƿ��ֹ�������� NAK
static bool no_retransmission = true;
static seq_nr retransmission_seq;

static void show_received_buffer()
{
    printf("\033[33m                                             ");
    for (int i = 0; i < 16; i++) {
        printf("|");
        if (arrived[i])
            printf("%2d", i % NR_BUFS);
        else
            printf("  ");
    }
    printf("|\033[0m\n");
}

static void put_frame(unsigned char* frame, int len) // ����֡�������
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

/* ��������֡ */
static void send_data_frame(seq_nr frame_nr)
{
    struct FRAME s;
    s.kind = FRAME_DATA;
    s.seq = frame_nr;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);   // ���ͷ���һ��Ҫȷ�ϵ�֡���
    memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN); // �����ݿ�����֡��
    printf("\033[32m");
    dbg_frame("Send DATA %d %d, ID %d |%d|%d|%d|\033[0m\n", s.seq, s.ack, *(short*)s.data, ack_expected, next_frame_to_send, phl_sq_len());
    put_frame((unsigned char*)&s, 3 + PKT_LEN);
    start_timer(frame_nr % NR_BUFS, DATA_TIMER); // ��������֡��ʱ��
    stop_ack_timer();
}

/* ���� ACK ֡ */
// �� send_data_frame ���ƣ������� ACK ֡ʱ����ҪЯ�����ݣ�������ֶ�δʹ��
static void send_ack_frame(void)
{
    struct FRAME s;
    s.kind = FRAME_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
    s.seq = next_frame_to_send;

    dbg_frame("Send ACK %d \n", s.ack);
    put_frame((unsigned char*)&s, 2);
    stop_ack_timer();
}

/* ���� NAK ֡ */
// NAK��ACK���ƣ���ACK֡��ack�ֶ�����һ��Ҫȷ�ϵ�֡��ţ���NAK֡��ack�ֶ�����һ��Ҫ���յ�֡���
static void send_nak_frame(void)
{
    struct FRAME s;
    s.kind = FRAME_NAK;

    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
    s.seq = 0; // seq �ֶ�δʹ��

    no_nak = false; // �������� NAK

    dbg_frame("Send NAK (ack=%d, ��������%d)\n", s.ack, s.ack + 1);
    put_frame((unsigned char*)&s, 2); // NAK ֡����Ϊ 2 (kind + ack)
    stop_ack_timer();
}

static void whether_send_half_ack()
{   
    struct FRAME s;
    s.kind = FRAME_HALF_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    seq_nr num = 0;
    for (seq_nr i = 0; i < NR_BUFS; i++) {
        if (arrived[i]) {
            s.data[num] = (i < too_far && i >= frame_expected) ? i : (i + NR_BUFS) % (MAX_SEQ + 1);
            num = num + 1;
        }
    }

    s.seq = num;

    if (num>0) {
        put_frame((unsigned char*)&s, 3 + num);
        dbg_frame("Send half ACK,ȷ�ϸ���Ϊ%d  ", num);
        for (seq_nr i = 0; i < num; i++) {
            printf("%d ", s.data[i]);
        }
        dbg_frame("\n");
    }
}

int main(int argc, char** argv)
{
    int event;
    seq_nr arg; // ����֡
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv);

    // ��ʼ��
    too_far = NR_BUFS;
    nbuffered = 0;
    phl_ready = 0;
    no_nak = true;
    memset(arrived, 0, sizeof(arrived));

    disable_network_layer();

    for (;;)
    {
        // --- �޸�: �� wait_for_event �Ĳ�������Ϊ seq_nr ---
        event = wait_for_event((int*)&arg); // ǿ������ת�������践�ص������

        switch (event)
        {
        case NETWORK_LAYER_READY:
            if (nbuffered < NR_BUFS)
            {
                // �����������ݲ����뷢�ͻ�����
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
            len = recv_frame((unsigned char*)&f, sizeof f);

            if (len < 5 || crc32((unsigned char*)&f, len) != 0)
            {
                switch (f.kind) {
                case FRAME_DATA:
                    dbg_event("**** CRC���� %d��֡����\n",f.seq);
                    break;
                case FRAME_ACK:
                    dbg_event("**** CRC���� ACK %d����\n",f.ack);
                    break;
                case FRAME_NAK:
                    dbg_event("**** CRC���� NAK %d %d����\n", f.ack, f.ack + 1);
                    break;
                }

                if (no_nak && f.kind != FRAME_HALF_ACK)
                {
                    // У������ҵ�ǰû��NAKʱ������NAK
                    send_nak_frame();
                    whether_send_half_ack();
                }
                break;
            }


            if (f.kind == FRAME_DATA)
            {
                printf("\033[33m");
                dbg_frame("�յ� DATA %d %d, ID %d             [%d,%d)\033[0m\n", f.seq, f.ack, *(short*)f.data, frame_expected, too_far);

                // ��Ԥ��֡���к�ʱ����NAK
                if (f.seq != frame_expected && no_nak)
                {
                    send_nak_frame();
                    whether_send_half_ack();
                }
                else
                {
                    // �յ�Ԥ��֡ʱ��ֹͣACK��ʱ��
                    start_ack_timer(ACK_TIMER);
                }

                if (between(frame_expected, f.seq, too_far)) // ����ڴ�����
                {
                    if (!arrived[f.seq % NR_BUFS]) // ����δ��ʱ
                    {

                        arrived[f.seq % NR_BUFS] = true;
                        memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);

                        show_received_buffer();//�����ã�������մ���

                        while (arrived[frame_expected % NR_BUFS])
                        {
                            // �����ύ����㣬���ʵ���ϲ���һ������
                            put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN);

                            no_nak = true;
                            arrived[frame_expected % NR_BUFS] = false;
                            inc(frame_expected);
                            inc(too_far);
                            start_ack_timer(ACK_TIMER);
                        }

                    }
                    else
                    {
                        // ���������߼������յ���ACK��ֱ������ʱ��
                        printf("\033[31m");
                        dbg_frame("DATA %d already received\033[0m\n", f.seq);
                        start_ack_timer(ACK_TIMER);
                    }
                }
                else
                {
                    // ��һ���ж�
                    dbg_frame("\n");//ɾ�˴�ӡ��ȱ��ʱ��
                    printf("\033[31m");
                    dbg_frame("DATA %d out of window [%d, %d)\033[0m\n", f.seq, frame_expected, too_far);
                    start_ack_timer(ACK_TIMER);
                }
            }

            // --- NAK ����
            if (f.kind == FRAME_NAK)
            {
                seq_nr missing_seq = (f.ack + 1) % (MAX_SEQ + 1); // �� ack �ƶ϶�ʧ֡
                dbg_frame("�յ� NAK (ack=%d), �ƶ϶�ʧ %d\n", f.ack, missing_seq);

                // �����ֽ����ж��ش�֡�Ƿ��ڵ�ǰ����
                if (between(ack_expected, missing_seq, next_frame_to_send))
                {
                    dbg_frame("�ش�֡ %d (�� NAK)\n", missing_seq);
                    send_data_frame(missing_seq);
                }
                else
                {
                    dbg_frame("�ƶ϶�ʧ��֡ %d ���ڴ��� [%d, %d) ��, ���� NAK\n", missing_seq, ack_expected, next_frame_to_send);
                }
                // break; // SR-2 û�� break���������Ӵ�ȷ��
            }

            // ���һ��ACK���յĵ��Ծ���
            if (f.kind == FRAME_ACK)
            {
                dbg_frame("�յ� ACK %d\n", f.ack);
            }

            if (f.kind == FRAME_HALF_ACK) {
                dbg_frame("�յ� half ACK, ȷ�ϸ���Ϊ%d  ", f.seq);
                dbg_event("ֹͣ��ʱ");
                for (seq_nr i = 0; i < f.seq; i++) {
                    printf("%d ", f.data[i]);
                    stop_timer((unsigned int)(f.data[i] % NR_BUFS));
                }
                dbg_frame("\n");
            }

            // ����ȷ����Ϣ
            // �ж��Ƿ��ڵ�ǰ�����ڣ�����ڴ����ڣ��򻬶�����
            while (between(ack_expected, f.ack, next_frame_to_send))
            {
                if (no_retransmission == false && retransmission_seq == ack_expected)
                    no_retransmission = true;

                nbuffered--;
                stop_timer(ack_expected % NR_BUFS);
                dbg_event("ͣ��%d ", ack_expected % NR_BUFS);
                inc(ack_expected);
            }
            dbg_frame("\n");
            break;

            // ��Ҫ�޸����ش����֡�ڲ��ڵ�ǰ���ڵ��߼�
        case DATA_TIMEOUT:
            if (between(ack_expected, arg, next_frame_to_send))
            {
                if (no_retransmission) {
                    dbg_event("---- DATA %d ��ʱ (���� wait_for_event) �ش���ʱ��֡\n", arg);
                    send_data_frame(arg);
                    retransmission_seq = arg;
                    no_retransmission = false;
                }
                else if (between(ack_expected, arg, retransmission_seq + 1)) {
                    dbg_event("---- DATA %d ��ʱ (���� wait_for_event) �ش���ʱ��֡\n", arg);
                    send_data_frame(arg);
                    retransmission_seq = arg;
                    no_retransmission = false;
                }
                else {
                    dbg_event("---- DATA %d ��ʱ����ʱ300ms\n", arg);
                    start_timer(arg, 300);
                }
            }
            else
            {
                if (no_retransmission) {
                    dbg_event("---- DATA %d ��ʱ���ش�\n", (arg + NR_BUFS) % (MAX_SEQ + 1));
                    send_data_frame((arg + NR_BUFS) % (MAX_SEQ + 1));
                    retransmission_seq = (arg + NR_BUFS) % (MAX_SEQ + 1);
                    no_retransmission = false;
                }
                else if (between(ack_expected, (arg + NR_BUFS) % (MAX_SEQ + 1), retransmission_seq + 1)) {
                    dbg_event("---- DATA %d ��ʱ���ش�\n", (arg + NR_BUFS) % (MAX_SEQ + 1));
                    send_data_frame((arg + NR_BUFS) % (MAX_SEQ + 1));
                    retransmission_seq = (arg + NR_BUFS) % (MAX_SEQ + 1);
                    no_retransmission = false;
                }
                else {
                    dbg_event("---- DATA %d ��ʱ����ʱ300ms\n", (arg + NR_BUFS) % (MAX_SEQ + 1));
                    start_timer(arg, 300);
                }
            }
            break;

        case ACK_TIMEOUT:
            dbg_event("---- ACK��ʱ\n");
            send_ack_frame();
            break;
        }

        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}