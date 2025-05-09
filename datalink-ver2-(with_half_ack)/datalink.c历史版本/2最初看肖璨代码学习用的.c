#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "protocol.h"

#define DATA_TIMER 5000
#define ACK_TIMER 280
#define MAX_SEQ 31
#define NR_BUFS ((MAX_SEQ + 1) / 2)

#define FRAME_DATA 0
#define FRAME_ACK 1
#define FRAME_NAK 2

#define inc(k)       \
    if (k < MAX_SEQ) \
        k++;         \
    else             \
        k = 0

typedef unsigned char seq_nr;

struct FRAME
{
    unsigned char kind;
    seq_nr ack;
    seq_nr seq;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};

//ȫ�ֱ���========================================

static seq_nr next_frame_to_send; // ���ͷ���һ��Ҫ���͵�֡���
static seq_nr ack_expected;       // ���ͷ���һ��Ҫȷ�ϵ�֡���
static seq_nr frame_expected;     // ���շ���һ��Ҫ���յ�֡���
static seq_nr too_far = NR_BUFS;                // ���շ���һ��Ҫ���յ�֡���

static unsigned char out_buf[NR_BUFS][PKT_LEN]; // ���ͷ�������
static unsigned char in_buf[NR_BUFS][PKT_LEN];  // ���շ�������
static bool arrived[NR_BUFS];                   // ���շ�������λͼ (�����Щ�������)

static int nbuffered;  // ���ͷ����������Ѵ�ŵ�֡��
static int phl_ready = 1;  // ������Ƿ�׼���ý�������
static bool no_nak = true; // �Ƿ��ֹ�������� NAK

//����========================================

static bool between(seq_nr a, seq_nr b, seq_nr c) // �ж�����Ƿ��ڴ�����
{
    return ((a <= b && b < c) || (c < a && a <= b) || (b < c && c < a));
}

static void put_frame(unsigned char* frame, int len) // ����֡�������
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

static void send_data_frame(seq_nr frame_nr)
{
    struct FRAME s;
    s.kind = FRAME_DATA;
    s.seq = frame_nr;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);   // ���ͷ���һ��Ҫȷ�ϵ�֡���
    memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN); // �����ݿ�����֡��

    dbg_frame("���� DATA %d %d, ID %d\n", s.seq, s.ack, *(short*)s.data);
    put_frame((unsigned char*)&s, 3 + PKT_LEN);
    start_timer(frame_nr % NR_BUFS, DATA_TIMER); // ��������֡��ʱ��
    stop_ack_timer();
}

// �� send_data_frame ���ƣ������� ACK ֡ʱ����ҪЯ�����ݣ�������ֶ�δʹ��
static void send_ack_frame(void)
{
    struct FRAME s;
    s.kind = FRAME_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    dbg_frame("���� ACK %d\n", s.ack);
    put_frame((unsigned char*)&s, 2);
    stop_ack_timer();
}

// NAK��ACK���ƣ���ACK֡��ack�ֶ�����һ��Ҫȷ�ϵ�֡��ţ���NAK֡��ack�ֶ�����һ��Ҫ���յ�֡���
static void send_nak_frame(void)
{
    struct FRAME s;
    s.kind = FRAME_NAK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    no_nak = false; // �������� NAK

    dbg_frame("���� NAK (ack=%d)\n", s.ack);
    put_frame((unsigned char*)&s, 2); // NAK ֡����Ϊ 2 (kind + ack)
    stop_ack_timer();
}

int main(int argc, char** argv)
{
    int event;
    seq_nr arg; // ����֡
    struct FRAME f;
    int len = 0;

    protocol_init(argc, argv);
    disable_network_layer();
    for (;;)
    {
        event = wait_for_event((int*)&arg); // ǿ������ת�������践�ص������

        switch (event)
        {
        case NETWORK_LAYER_READY:
            get_packet(out_buf[next_frame_to_send % NR_BUFS]);
            nbuffered++;
            send_data_frame(next_frame_to_send);
            inc(next_frame_to_send);
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char*)&f, sizeof f);

            if (len < 5 || crc32((unsigned char*)&f, len) != 0)
            {
                dbg_event("**** CRC����\n");
                if (no_nak)
                    send_nak_frame();
                break;
            }

            if (f.kind == FRAME_DATA)
            {
                dbg_frame("�յ� DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
                dbg_frame("frame_expected = %d, too_far = %d\n", frame_expected, too_far);
                // ��Ԥ��֡���к���δ����NAKʱ������NAK
                if (f.seq != frame_expected && no_nak)
                    send_nak_frame();
                else
                    start_ack_timer(ACK_TIMER);

                if (between(frame_expected, f.seq, too_far)) // ����ڴ�����
                {
                    if (!arrived[f.seq % NR_BUFS]) // ����δ��ʱ
                    {
                        arrived[f.seq % NR_BUFS] = true;
                        memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);
                        while (arrived[frame_expected % NR_BUFS])
                        {
                            put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN);

                            no_nak = true;
                            arrived[frame_expected % NR_BUFS] = false;
                            inc(frame_expected);
                            inc(too_far);
                            start_ack_timer(ACK_TIMER);//MYM���������ʲô�ã��α���Ҳ����ôд��
                        }
                    }
                    else
                    {
                        // ���������߼������յ���ACK��ֱ������ʱ��
                        // MYM:�����֧�������ڵ��� ָ���飺����ǰ�����Ķ�ʱ��δ��ʱ֮ǰ����ִ�� start_ack_timer()���ã���ʱ������Ȼ������ǰ��ʱ�����ò�����ʱ�¼� ACK_TIMEOUT
                        dbg_frame("DATA %d ���յ���\n", f.seq);
                        start_ack_timer(ACK_TIMER);
                    }
                }
                else
                {
                    //MYM�������֧����������ǵ���ʱ��ʾ���������˴����⣬start_ack_timer()�������ָ������˵����������
                    dbg_frame("DATA %d �ڴ����� [%d, %d)\n", f.seq, frame_expected, too_far);
                    start_ack_timer(ACK_TIMER);
                }
            }

            if (f.kind == FRAME_NAK)
            {
                seq_nr missing_seq = (f.ack + 1) % (MAX_SEQ + 1); // �� ack �ƶ϶�ʧ֡
                dbg_frame("�յ� NAK (ack=%d), �ƶ϶�ʧ %d\n", f.ack, missing_seq);

                // �����ֽ����ж��ش�֡�Ƿ��ڵ�ǰ���� MYM���α�Ҳ������жϣ������Ϊʲô
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

            if (f.kind == FRAME_ACK)
            {
                dbg_frame("�յ� ACK %d\n", f.ack);
            }

            // �ж��Ƿ��ڵ�ǰ�����ڣ�����ڴ����ڣ��򻬶�����
            while (between(ack_expected, f.ack, next_frame_to_send))
            {
                nbuffered--;
                stop_timer(ack_expected % NR_BUFS);
                inc(ack_expected);
            }
            break;

        // ��Ҫ�޸����ش����֡�ڲ��ڵ�ǰ���ڵ��߼�
        case DATA_TIMEOUT:
            dbg_event("---- DATA %d ��ʱ (���� wait_for_event)\n", arg);
            if (between(ack_expected, arg, next_frame_to_send))
            {
                dbg_event("---- �ش���ʱ��֡ %d\n", arg);
                send_data_frame(arg);
            }
            else
            {
                dbg_event("---- ��ʱ֡ %d ���ڵ�ǰ���� [%d, %d) ��, �ݲ��ش�\n", arg, ack_expected, next_frame_to_send);
                send_data_frame(arg + NR_BUFS); // ������Ϊ�˱�����ѭ����ֱ���ش��������֡
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