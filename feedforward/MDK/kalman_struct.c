#include "kalman_struct.h"

struct Kalman_Typedef
{
    /*���ö�*/
    float LastP;//�ϴι���Э����
    float Now_P;//��ǰ����Э����
    float out;//�������˲������
    float Kg;//����������
	  float Q;
	  float R;
}klm;

float KalmanFilter(float input)
{
		
	float out;
    //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
    klm.Now_P = klm.LastP + klm.Q;
    //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
    klm.Kg = klm.Now_P / (klm.Now_P + klm.R);
    //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
    klm.out = klm.out + klm.Kg * (input -klm.out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
    //����Э�����: ���ε�ϵͳЭ����� klm->LastP Ϊ��һ������׼����
    klm.LastP = (1-klm.Kg) * klm.Now_P;
	out=klm.out;
	
	return (klm.out);
}

void Kalman_Init(void)//�¶�klm_Q=0.01 klm_R=0.25
{
	klm.LastP=0.02;		//�ϴι���Э����
	klm.Now_P=0;			//��ǰ����Э����
	klm.out=0;				//�������˲������
	klm.Kg=0;				//����������
	klm.Q=0;			//Q:��������Э���� Q�������˲��������ƽ���̶ȣ�QԽСԽƽ��;
	klm.R=0;			//R:�۲�����Э���� R���������˲����������ʵ�����ߵ�����̶ȣ�RԽСԽ�ӽ�(����Խ��)
}

