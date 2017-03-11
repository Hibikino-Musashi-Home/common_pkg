//--------------------------------------------------
//オブジェクト数カウントROSノード
//
//author: Yutaro ISHIDA
//date: 16/03/14
//
//TODO: パラメータを定数化する
//TODO: オブジェクトのtfに回転を与える
//--------------------------------------------------


#include <common_pkg/common_include.h>
#include <common_pkg/common_function.h>


//pcl::visualization::CloudViewer viewer("viewer");


//--------------------------------------------------
//オブジェクト数カウントクラス
//--------------------------------------------------
class ObjCnt{
private:
    //ROSノードのハンドル
    ros::NodeHandle nh_;

    int dbg_sm_flow_;
    int dbg_speech_onlyspeech_;
    double iarm_obj_height_;

    ros::Subscriber sub_smpc_cam_;
    ros::Publisher pub_smpc_pt_;
    ros::Publisher pub_smpc_plane_;
    ros::Publisher pub_smpc_objs_;

    int sub_smpc_cam_cnt_;

    int pills_cnt_;

    //オブジェクト数カウント用変数
    //例: 1個目のオブジェクト(キー番号: 0)
    //オブジェクト位置 objs_pos_[0][0] = x[m], objs_pos_[0][1]= y[m], objs_pos_[0][2] = z[m]
    //投票数 objs_poll_[0] = 1～?
    vector< vector<float> > objs_pos_; //オブジェクト位置
    vector<int> objs_poll_; //投票数

    CommonFunction* CommonFunction_;

public:
    //--------------------------------------------------
    //コンストラクタ
    //--------------------------------------------------
    ObjCnt(){
        nh_.getParam("/param/dbg/sm/flow", dbg_sm_flow_);
        nh_.getParam("/param/dbg/speech/onlyspeech", dbg_speech_onlyspeech_);
        if(dbg_sm_flow_ == 1 || dbg_speech_onlyspeech_ == 1){
            exit(0);
        }

        nh_.getParam("/param/iarm/obj/height", iarm_obj_height_);

        sub_smpc_cam_ = nh_.subscribe("/camera/depth_registered/points", 1, &ObjCnt::subf_smpc_cam, this);
        pub_smpc_pt_ = nh_.advertise<sensor_msgs::PointCloud2>("/obj_count/pc/pt", 1);         
        pub_smpc_plane_ = nh_.advertise<sensor_msgs::PointCloud2>("/obj_count/pc/plane", 1);
        pub_smpc_objs_ = nh_.advertise<sensor_msgs::PointCloud2>("/obj_count/pc/objs", 1);
      
        sub_smpc_cam_cnt_ = 0;

        pills_cnt_ = 0;

        CommonFunction_ = new CommonFunction(nh_);

        CommonFunction_->commonf_actionf_speech_multi(nh_, "到着しました。");

        CommonFunction_->commonf_actionf_cam_lift(nh_, iarm_obj_height_ - 0.28);
        CommonFunction_->commonf_pubf_cam_tilt(nh_, 0.785);

        ros::Duration(2).sleep();
    }


    //--------------------------------------------------
    //デストラクタ
    //--------------------------------------------------
    ~ObjCnt(){
    }


    //--------------------------------------------------
    //ポイントクラウドSubscribe関数
    //--------------------------------------------------
    void subf_smpc_cam(const sensor_msgs::PointCloud2ConstPtr& i_smpc_ptr){ //smpc_ptr: sensor_msgs::PointCloud2ConstPtr&
        //複数箇所でオブジェクトをカウントする
        sub_smpc_cam_cnt_++;
        if(sub_smpc_cam_cnt_ == 5 || sub_smpc_cam_cnt_ == 10 || sub_smpc_cam_cnt_ == 15 || sub_smpc_cam_cnt_ == 20){
            //CommonFunction_->commonf_actionf_speech_multi(nh_, "１５センチ横に移動します。");
            CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, -0.15, 0, 0, 0, 0);
            ros::Duration(1).sleep();
            CommonFunction_->commonf_pubf_cmd_vel(nh_, 0, 0, 0, 0, 0, 0);
            ros::Duration(2).sleep();
            return;
        }
        else if(sub_smpc_cam_cnt_ == 25){
            string half2full[10] = {"０", "１", "２", "３", "４", "５", "６", "７", "８", "９"};
            //CommonFunction_->commonf_actionf_speech_single(nh_, "全部で" + half2full[pills_cnt_] + "個のオブジェクトがあります。");
            CommonFunction_->commonf_pubf_cam_tilt(nh_, 0.000);
            exit(0);
        }


        //sensor_msgs::PointCloud2ConstPtr&からpcl::PointCloud<pcl::PointXYZRGB>に変換
        pcl::PointCloud<pcl::PointXYZRGB> i_pc; //pc: PointCloud<>
        pcl::fromROSMsg (*i_smpc_ptr, i_pc);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(i_pc)); //pc_ptr: PointCloud<>::Ptr

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objs_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        
        //viewer.showCloud(_pc_ptr); //何故かThinkPad T450では動かない


        //ダウンサンプリング
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(i_pc_ptr);
        vg.setLeafSize(0.01, 0.01, 0.01); //ダウンサンプリングの間隔[m]
        vg.filter(*plane_pc_ptr);


        //パススルーフィルタでポイントクラウドの領域を制限
        pcl::PassThrough<pcl::PointXYZRGB> pass_th;

        pass_th.setInputCloud(plane_pc_ptr);
        pass_th.setFilterFieldName("x");
        pass_th.setFilterLimits(-0.1, 0.3);
        pass_th.setFilterLimitsNegative(false);
        pass_th.filter(*plane_pc_ptr);

        pass_th.setInputCloud(plane_pc_ptr);
        pass_th.setFilterFieldName("y");
        pass_th.setFilterLimits(-0.1, 0.2);
        pass_th.setFilterLimitsNegative(false);
        pass_th.filter(*plane_pc_ptr);

        pass_th.setInputCloud(plane_pc_ptr);
        pass_th.setFilterFieldName("z");
        pass_th.setFilterLimits(0.4, 0.8);
        pass_th.setFilterLimitsNegative(false);
        pass_th.filter(*plane_pc_ptr);

        //パススルーフィルタを通したポイントクラウドをPublish
        sensor_msgs::PointCloud2 o_smpc_pt;
        pcl::toROSMsg(*plane_pc_ptr, o_smpc_pt);
        o_smpc_pt.header.frame_id = "camera_depth_optical_frame";
        pub_smpc_pt_.publish(o_smpc_pt);


        //ポイントクラウドをコピー
        pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*plane_pc_ptr, *objs_pc_ptr);  


        //平面検出
        pcl::ModelCoefficients::Ptr model_coef(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);

        //セグメンテーションオブジェクト生成
        pcl::SACSegmentation<pcl::PointXYZRGB> sac_seg;
        //セグメンテーションオブジェクト必須オプション指定
        sac_seg.setModelType(pcl::SACMODEL_PLANE);
        sac_seg.setMethodType(pcl::SAC_RANSAC);
        sac_seg.setDistanceThreshold(0.01); //[m]
        //セグメンテーションオブジェクトオプション指定
        sac_seg.setOptimizeCoefficients(true);
        //セグメンテーションオブジェクトにポイントクラウドを入力して平面検出
        sac_seg.setInputCloud(objs_pc_ptr);
        sac_seg.segment (*indices, *model_coef);

        //平面が検出されない場合
        if(indices->indices.size() == 0){
            ROS_INFO("[obj_proc]: There is no plane.");
            return;
        }

        //平面を青色にする
        for(size_t i = 0; i < indices->indices.size(); ++i){
            plane_pc_ptr->points[indices->indices[i]].r = 0;
            plane_pc_ptr->points[indices->indices[i]].g = 0;
            plane_pc_ptr->points[indices->indices[i]].b = 255;
        }

        //平面を青色にしたポイントクラウドをパブリッシュ
        sensor_msgs::PointCloud2 o_smpc_plane;
        pcl::toROSMsg(*plane_pc_ptr, o_smpc_plane);
        o_smpc_plane.header.frame_id = "camera_depth_optical_frame";
        pub_smpc_plane_.publish(o_smpc_plane);


        //平面除去
        pcl::ExtractIndices<pcl::PointXYZRGB> ext;
        ext.setInputCloud(objs_pc_ptr);
        ext.setIndices(indices);
        ext.setNegative(true); //trueにすると平面を除去、falseにすると平面以外を除去
        ext.filter(*objs_pc_ptr);

        //複数のオブジェクトが含まれたポイントクラウドをパブリッシュ
        sensor_msgs::PointCloud2 o_smpc_objs;
        pcl::toROSMsg(*objs_pc_ptr, o_smpc_objs);
        o_smpc_objs.header.frame_id = "camera_depth_optical_frame";
        pub_smpc_objs_.publish(o_smpc_objs);


        //クラスタリング
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        kdtree->setInputCloud(objs_pc_ptr);

        std::vector<pcl::PointIndices> cluster_indices; //クラスター情報

        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_ext;
        ec_ext.setClusterTolerance(0.02); //クラスターを分ける閾値[m]
        ec_ext.setMinClusterSize(100); //最小のクラスターの値を設定
        ec_ext.setMaxClusterSize(1000); //最大のクラスターの値を設定
        ec_ext.setSearchMethod(kdtree); //検索に使用する手法を指定
        ec_ext.setInputCloud(objs_pc_ptr); //点群を入力
        ec_ext.extract(cluster_indices); //クラスター情報を出力


        //クラスタを1塊ごとに出力
        for(std::vector<pcl::PointIndices>::const_iterator it1 = cluster_indices.begin(); it1 != cluster_indices.end(); ++it1){
            //オブジェクトの各座標における最小、最大値
            double obj_pc_x_min = objs_pc_ptr->points[*it1->indices.begin()].x;
            double obj_pc_x_max = objs_pc_ptr->points[*it1->indices.begin()].x;
            double obj_pc_y_min = objs_pc_ptr->points[*it1->indices.begin()].y;
            double obj_pc_y_max = objs_pc_ptr->points[*it1->indices.begin()].y;
            double obj_pc_z_min = objs_pc_ptr->points[*it1->indices.begin()].z;
            double obj_pc_z_max = objs_pc_ptr->points[*it1->indices.begin()].z;
        
            for(std::vector<int>::const_iterator it2 = it1->indices.begin(); it2 != it1->indices.end(); it2++){
                if(obj_pc_x_min > objs_pc_ptr->points[*it2].x) obj_pc_x_min = objs_pc_ptr->points[*it2].x;
                if(obj_pc_x_max < objs_pc_ptr->points[*it2].x) obj_pc_x_max = objs_pc_ptr->points[*it2].x;
                if(obj_pc_y_min > objs_pc_ptr->points[*it2].y) obj_pc_y_min = objs_pc_ptr->points[*it2].y;
                if(obj_pc_y_max < objs_pc_ptr->points[*it2].y) obj_pc_y_max = objs_pc_ptr->points[*it2].y;
                if(obj_pc_z_min > objs_pc_ptr->points[*it2].z) obj_pc_z_min = objs_pc_ptr->points[*it2].z;
                if(obj_pc_z_max < objs_pc_ptr->points[*it2].z) obj_pc_z_max = objs_pc_ptr->points[*it2].z;            
            }

            //オブジェクトの各座標における中心座標
            double obj_pc_x_ave = (obj_pc_x_min + obj_pc_x_max) / 2.0;
            double obj_pc_y_ave = (obj_pc_y_min + obj_pc_y_max) / 2.0;
            double obj_pc_z_ave = (obj_pc_z_min + obj_pc_z_max) / 2.0;


            tf::TransformListener tf_listener;
            tf::StampedTransform tf_stamped_transform;                            

            tf::TransformBroadcaster tf_broadcaster;
            tf::Transform tf_transform;
            tf_transform.setOrigin(tf::Vector3(obj_pc_x_ave , obj_pc_y_ave, obj_pc_z_min));
            tf::Quaternion tf_quaternion;
            tf_quaternion.setRPY(0, 0, 0);
            tf_transform.setRotation(tf_quaternion);

            while(ros::ok()){
                tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "camera_depth_optical_frame", "obj"));
                try{
                    tf_listener.lookupTransform("map", "obj", ros::Time(0), tf_stamped_transform);    
                }
                catch(tf::TransformException){
                    continue;
                }
                break;
            }


            //オブジェクト数カウント
            bool flag_poll = false; //投票が行われたかチェックする
            for(size_t i = 0; i < objs_pos_.size(); i++){
                //ほぼ同じ座標か確認
                if(abs(objs_pos_[i][0] - tf_stamped_transform.getOrigin().x()) < 0.1 &&
                   abs(objs_pos_[i][1] - tf_stamped_transform.getOrigin().y()) < 0.1 &&
                   abs(objs_pos_[i][2] - tf_stamped_transform.getOrigin().z()) < 0.1){
                    objs_poll_[i]++;
                    flag_poll = true;

                    //3回投票された場合
                    if(objs_poll_[i] == 3){
                        pills_cnt_++;
                        nh_.setParam("/param/pills/cnt", pills_cnt_);
                        ROS_INFO("There are %d pills", pills_cnt_);
                        string half2full2[10] = {"０", "１", "２", "３", "４", "５", "６", "７", "８", "９"};
                        CommonFunction_->commonf_actionf_speech_single(nh_, "これは左から" + half2full2[pills_cnt_] + "個目の薬です。");
            
                    }

                    break;
                }
            }

            //投票されなかった時、新規投票
            if(flag_poll == false){
                vector<float> buf_pos(3);
                buf_pos[0] = tf_stamped_transform.getOrigin().x();
                buf_pos[1] = tf_stamped_transform.getOrigin().y();
                buf_pos[2] = tf_stamped_transform.getOrigin().z();
                objs_pos_.push_back(buf_pos);
                objs_poll_.push_back(1);
            }
        }
    }
};


//--------------------------------------------------
//メイン関数
//--------------------------------------------------
int main (int argc, char** argv){
    ros::init (argc, argv, "obj_proc");

    ObjCnt ObjCnt;

    while(ros::ok()){
        ros::spinOnce(); 
    }
}
