//--------------------------------------------------
//人物追跡ROSノード
//
//author: Yutaro ISHIDA
//date: 16/03/12
//--------------------------------------------------


#include <common_pkg/common_include.h>
#include <common_pkg/common_function.h>


//--------------------------------------------------
//定数宣言
//--------------------------------------------------
#define DEPTH_W 640 //深度画像の幅
#define DEPTH_H 480 //深度画像の高さ

#define PARTICLE_NUM 4000 //パーティクルの数

#define AOI_CENTER_BASE 0.9 //注目領域(中心)の初期距離[m]
#define AOI_FRONT_WIDTH 0.15 //注目領域(手前)の幅[m]
#define AOI_BACK_WIDTH 0.15 //注目領域(奥)の幅[m]
#define AOI_CENTER_LIM_MIN 0.6 //注目領域(中心)の最小距離[m]
#define AOI_CENTER_LIM_MAX 1.2 //注目領域(中心)の最小距離[m]

#define CAM_PAN_LIM_MIN -0.872 //camのpanの最小角度[rad] -50[deg]
#define CAM_PAN_LIM_MAX 0.872 //camのpanの最小角度[rad] 50[deg]


//--------------------------------------------------
//グローバル変数宣言
//--------------------------------------------------
//パーティクル構造体
struct particle{
    double w; //重み
	int sv[4]; //状態ベクトル    
};

particle p[PARTICLE_NUM];

int sv_upper[4] = {DEPTH_W, DEPTH_H, 10, 10}; //状態変数の上限
int sv_lower[4] = {0, 0, -10, -10}; //状態変数の下限
int noise_lim[4] = {30, 30, 10, 10}; //ノイズの最大値


//--------------------------------------------------
//パーティクルの初期化
//--------------------------------------------------
void particle_init(){
    for(int i = 0; i < PARTICLE_NUM; i++){    
        p[i].w = 0.0; //重みを0にする
        p[i].sv[0] = 0; //状態変数を0にする
        p[i].sv[1] = 0;
        p[i].sv[2] = 0;
        p[i].sv[3] = 0;
    }
}


//--------------------------------------------------
//パーティクルフィルタの初期化
//--------------------------------------------------
void pfilter_init(){
    for(int i = 0; i < PARTICLE_NUM; i++){
        p[i].w = 1.0 / (double)PARTICLE_NUM; //パーティクルに初期重みを付ける
        p[i].sv[0] = rand() % (sv_upper[0] - sv_lower[0]) + sv_lower[0]; //パーティクルの初期状態変数を決める
        p[i].sv[1] = rand() % (sv_upper[1] - sv_lower[1]) + sv_lower[1];
        p[i].sv[2] = rand() % (sv_upper[2] - sv_lower[2]) + sv_lower[2];
        p[i].sv[3] = rand() % (sv_upper[3] - sv_lower[3]) + sv_lower[3];
    }
}


//--------------------------------------------------
//重みに基づきリサンプリング
//--------------------------------------------------
void pfilter_resample(){
    //累積重み
    double w[PARTICLE_NUM];
    w[0] = p[0].w;
    for(int i = 1; i < PARTICLE_NUM; i++){
        w[i] = w[i - 1] + p[i].w;
    }

    //パーティクルのコピー
    particle cp[PARTICLE_NUM];
    for(int i = 0; i < PARTICLE_NUM; i++){
        cp[i] = p[i];
    }

    //コピーに基づきリサンプリング
    double num_rand;
    for(int i = 0; i < PARTICLE_NUM; i++){
        num_rand = (double)(rand() % 10000) / 10000.0;
        for(int j = 0; j < PARTICLE_NUM; j++){
            if(num_rand < w[j]){
                p[i].w = 0.0;
                p[i].sv[0] = cp[j].sv[0];
                p[i].sv[1] = cp[j].sv[1];
                p[i].sv[2] = cp[j].sv[2];
                p[i].sv[3] = cp[j].sv[3];
                break;
            }
        }
    }
}


//--------------------------------------------------
//パーティクルに状態遷移モデルを適用(事前推定)
//--------------------------------------------------
void pfilter_predict(){
    int noise[4];

    for(int i = 0; i < PARTICLE_NUM; i++){
        //ノイズを生成
        noise[0] = rand() % (noise_lim[0] * 2) - noise_lim[0];
        noise[1] = rand() % (noise_lim[1] * 2) - noise_lim[1];
        noise[2] = rand() % (noise_lim[2] * 2) - noise_lim[2];
        noise[3] = rand() % (noise_lim[3] * 2) - noise_lim[3];

        //ノイズを加えて等速直線運動
        p[i].sv[0] += p[i].sv[2] + noise[0];
        p[i].sv[1] += p[i].sv[3] + noise[1];
        p[i].sv[2] += noise[2];
        p[i].sv[3] += noise[3];

        //上限より大きいとき、上限値にする
        if(p[i].sv[0] >= sv_upper[0]) p[i].sv[0] = sv_upper[0];
        if(p[i].sv[1] >= sv_upper[1]) p[i].sv[1] = sv_upper[1];
        if(p[i].sv[2] >= sv_upper[2]) p[i].sv[2] = sv_upper[2];
        if(p[i].sv[3] >= sv_upper[3]) p[i].sv[3] = sv_upper[3];        

        //下限より小さい時、下限値にする
        if(p[i].sv[0] < sv_lower[0]) p[i].sv[0] = sv_lower[0];
        if(p[i].sv[1] < sv_lower[1]) p[i].sv[1] = sv_lower[1];
        if(p[i].sv[2] < sv_lower[2]) p[i].sv[2] = sv_lower[2];
        if(p[i].sv[3] < sv_lower[3]) p[i].sv[3] = sv_lower[3];
    }
}


//--------------------------------------------------
//パーティクルの重み付け
//--------------------------------------------------
void pfilter_weight(unsigned char* depth_8UC1_ptr){
    for(int i = 0; i < PARTICLE_NUM; i++){
        if(depth_8UC1_ptr[p[i].sv[1] * DEPTH_W + p[i].sv[0]] == 255){
            p[i].w = 1.0; //人物領域は重みを大きく付ける
        }
        else{
            p[i].w = 0.0001; //人間領域以外は重みを小さく付ける
        }
    }

    //正規化
    double sum_w = 0.0;
    for(int i = 0; i < PARTICLE_NUM; i++){
        sum_w += p[i].w;
    }
    for(int i = 0; i < PARTICLE_NUM; i++){
        p[i].w /= sum_w;
    }
}


//--------------------------------------------------
//パーティクルの重み付き平均を推定結果として出力
//--------------------------------------------------
void pfilter_measure(int* res_sv){
    double sv[4];

    sv[0] = 0.0;
    sv[1] = 0.0;
    sv[2] = 0.0;
    sv[3] = 0.0;

    for(int i = 0; i < PARTICLE_NUM; i++){
        sv[0] += (double)p[i].sv[0] * p[i].w;
        sv[1] += (double)p[i].sv[1] * p[i].w;
        sv[2] += (double)p[i].sv[2] * p[i].w;
        sv[3] += (double)p[i].sv[3] * p[i].w;
    }

    *res_sv = (int)sv[0];
    *(res_sv + 1) = (int)sv[1];
    *(res_sv + 2) = (int)sv[2];
    *(res_sv + 3) = (int)sv[3];
}


//--------------------------------------------------
//人物追跡クラス
//--------------------------------------------------
class Img_FollowParson{
private:
    ros::NodeHandle nh_;

    int dbg_sm_flow_;
    int dbg_speech_onlyspeech_;        
  
    image_transport::ImageTransport img_t_;
    image_transport::Subscriber img_sub_depth_;
    image_transport::Publisher img_pub_follow_;
  
    float aoi_front_; //注目領域制限手前
    float aoi_back_; //注目領域制限奥

    bool flag_first_;

    float cam_pan_;

    CommonFunction* CommonFunction_;


public:
    Img_FollowParson():img_t_(nh_){
        nh_.getParam("/param/dbg/sm/flow", dbg_sm_flow_);
        nh_.getParam("/param/dbg/speech/onlyspeech", dbg_speech_onlyspeech_);
        if(dbg_sm_flow_ == 1 || dbg_speech_onlyspeech_ == 1){
            exit(0);
        }

        img_sub_depth_ = img_t_.subscribe("/camera/depth_registered/image_raw", 1, &Img_FollowParson::subf_depth, this);
        img_pub_follow_ = img_t_.advertise("/img/followparson/follow", 1);

        aoi_front_ = AOI_CENTER_BASE - AOI_FRONT_WIDTH;
        aoi_back_ = AOI_CENTER_BASE + AOI_BACK_WIDTH;

        flag_first_ = 1;

        cam_pan_ = 0;

        particle_init();
        pfilter_init();

        CommonFunction_ = new CommonFunction(nh_);

        CommonFunction_->commonf_actionf_speech_single(nh_, "１メートル前に立ってください。");
    }


    ~Img_FollowParson(){
    }


    void subf_depth(const sensor_msgs::ImageConstPtr& depth){
        cv::Mat depth_32FC1;
        unsigned char* depth_8UC1_ptr;
        depth_8UC1_ptr = (unsigned char*)malloc(DEPTH_W * DEPTH_H);

        int aoi_area = 0; //注目領域の面積

        int res_sv[4];

        clock_t c1,c2; //処理時間計測用


        //openni2.launchでパブリッシュされる/camera/depth_registered/image_rawは32FC1
        cv_bridge::CvImagePtr cv_img_ptr;
        try{
            cv_img_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        depth_32FC1 = cv_img_ptr->image; //[m]単位の深度画像


        for(int y = 0; y < DEPTH_H; y++){
            for(int x = 0; x < DEPTH_W; x++){
                if(y > DEPTH_H - 100){
                    depth_8UC1_ptr[y * DEPTH_W + x] = 0;
                    depth_32FC1.at<float>(y, x) = 0;
                }
                else{
                    if(depth_32FC1.at<float>(y, x) > aoi_front_ && depth_32FC1.at<float>(y, x) < aoi_back_){
                        depth_8UC1_ptr[y * DEPTH_W + x] = 255;
                        //depth_32FC1.at<float>(y, x) = 5;
                        aoi_area++;
                    }
                    else{
                        depth_8UC1_ptr[y * DEPTH_W + x] = 0;
                        depth_32FC1.at<float>(y, x) = 0;
                    }
                }
            }
        }


        //c1 = clock(); //処理開始時間    
        pfilter_resample();
        pfilter_predict();
        pfilter_weight(depth_8UC1_ptr);
        pfilter_measure(res_sv);
        //c2 = clock(); //処理終了時間


        if(depth_32FC1.at<float>(res_sv[1], res_sv[0]) > AOI_CENTER_LIM_MIN){
            aoi_front_ = depth_32FC1.at<float>(res_sv[1], res_sv[0]) - AOI_FRONT_WIDTH;
            aoi_back_ = depth_32FC1.at<float>(res_sv[1], res_sv[0]) + AOI_BACK_WIDTH;
        }


        if(aoi_area > 10000 && (aoi_front_ + aoi_back_) / 2 > AOI_CENTER_LIM_MIN && (aoi_front_ + aoi_back_) / 2 < AOI_CENTER_LIM_MAX){
            if(flag_first_ == 1){
                flag_first_ = 0;
                CommonFunction_->commonf_actionf_sound_effect_multi(nh_, "follow");
            }


            if(res_sv[0] < (DEPTH_W / 2) - 40){
                if(cam_pan_ < CAM_PAN_LIM_MAX){
                    cam_pan_= cam_pan_ + 0.035;
                }
            }
            else if(res_sv[0] > (DEPTH_W / 2) + 40){
                if(cam_pan_ > CAM_PAN_LIM_MIN){
                    cam_pan_ = cam_pan_ - 0.035;
                }
            }

            CommonFunction_->commonf_pubf_cam_pan(nh_, cam_pan_);


            if((aoi_front_ + aoi_back_) / 2 > 0.7){
                if(cam_pan_ < -0.436){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0.4, 0, 0, 0, 0, -0.436);
                }
                else if(cam_pan_ > 0.436){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0.4, 0, 0, 0, 0, 0.436);
                }
                else if(cam_pan_ < -0.261){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0.4, 0, 0, 0, 0, -0.261);
                }
                else if(cam_pan_ > 0.261){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0.4, 0, 0, 0, 0, 0.261);
                }
                else{        
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0.5, 0, 0, 0, 0, 0);
                }
            }
            else{
                if(cam_pan_ < -0.436){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, 0, 0, 0, 0, -0.436);
                }
                else if(cam_pan_ > 0.436){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, 0, 0, 0, 0, 0.436);
                }
                else if(cam_pan_ < -0.261){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, 0, 0, 0, 0, -0.174);
                }
                else if(cam_pan_ > 0.261){
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, 0, 0, 0, 0, 0.174);
                }
                else{
                    CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, 0, 0, 0, 0, 0);
                }
            }


            cv::circle(depth_32FC1, cv::Point(res_sv[0], res_sv[1]), 10, cv::Scalar(5), 8, 8);

            for(int i = 0; i < PARTICLE_NUM; i++){
			    cv::circle(depth_32FC1, cv::Point(p[i].sv[0], p[i].sv[1]), 1, cv::Scalar(5));
		    }
        }
        else{
            CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, 0, 0, 0, 0, 0);
        }

        cv_img_ptr->image = depth_32FC1;
        img_pub_follow_.publish(cv_img_ptr->toImageMsg());
    }
};


//--------------------------------------------------
//メイン関数
//--------------------------------------------------
int main(int argc, char** argv){
    ros::init(argc, argv, "img_followparson");

    Img_FollowParson img_followParson;

    while(ros::ok()){
        ros::spinOnce(); 
    }
}
