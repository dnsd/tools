#include <iostream>
#include <ssm.hpp>
#include <cmath>
#include <fstream>
#include <sys/time.h>
#include "decision.h"

#include <vector>
#include <algorithm>
#include <deque>
#include <iterator>

using namespace std;

#define FILENAME "debug.csv"

//-ファイル読み込み用-//
FILE *fp; //ファイルポインタ
ofstream ofs;

//-SSM-//
SSMApi<LS3D> OBJECT("OBJECT", 1);
SSMApi<ORDER> DORDER("ORDER", 30);
SSMApi<AREATH> TH("AREATH", 40);

int main (int argc, char **argv)
{
    //-SSM-//
    initSSM();
    OBJECT.open(SSM_READ);
    DORDER.create(5.0, 1.0);
    TH.create(5.0, 1.0);

    //-クラス-//
    Step ped;
    BEAMANGLE beam_angle;

    //-初期設定-//
    read_beta_data(beam_angle);

    AABB aabb1;
    aabb1.min[0] = 1000.0;
    aabb1.min[1] = -500.0;
    aabb1.min[2] = 0.0;
    aabb1.max[0] = 2000.0;
    aabb1.max[1] = 500.0;
    aabb1.max[2] = 2000.0;
    AreaAABB area1(aabb1, 20, 10, 15);
    AREATH TH_SSM;
    area1.calAreaTh(beam_angle, TH_SSM);

    // 出力の設定
    ofs.open(FILENAME);
    ofs.precision(16);
    cout.precision(16);
    //-初期設定おわり-//

    while(1) //データ取得ループ
    {
        if (OBJECT.readNew())//if(OBJECTreadNew)
        {
            double time_1 = get_time();

            //-読み込みデータのセット-//
            ped.set_data(OBJECT.data.det, OBJECT.data.dist, OBJECT.data.x, OBJECT.data.y, OBJECT.data.z);

            // 開き命令
            DORDER.data.order = judge_open_mode(area1.judgeOpen(ped));

            // TH書き込み
            for (int i = 0; i < STEP_NUM; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    TH.data.U_min[i][j] = TH_SSM.U_min[i][j];
                    TH.data.U_max[i][j] = TH_SSM.U_max[i][j];
                    TH.data.D_min[i][j] = TH_SSM.D_min[i][j];
                    TH.data.D_max[i][j] = TH_SSM.D_max[i][j];
                    // TH.data.U_min[i][j] = 0.0;
                    // TH.data.U_max[i][j] = area1.beam_range_U[i][j];
                    // TH.data.D_min[i][j] = 0.0;
                    // TH.data.D_max[i][j] = area1.beam_range_D[i][j];
                }
            }

            // SSM書き込み
            DORDER.write();
            TH.write(); // デバッグ用

            //-結果のファイル出力-//
            for (int i = 0; i < STEP_NUM; ++i)
            {
                // ofs << TH.data.U_min[i][0] << endl;
                // ofs << area1.beam_range_U[i][0] << endl;
            }

            double time_2 = get_time();
            usleep(FREQ*1000000 - (time_2 - time_1)*1000000);
            double time_3 = get_time();

            //-結果の標準出力-//
            cout << "open_mode_door = " << DORDER.data.order << endl; //開き判定のコンソール出力
        }else{  //if(OBJECTreadNew)
            usleep(1000); //CPU使用率100％対策
        } 
    }
    //-SSMクローズ-//
    OBJECT.close();
    endSSM();
    return 0;
}
