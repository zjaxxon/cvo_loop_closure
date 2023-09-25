#include "CvoLoopClosing.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <UnifiedCvo-0.1/dataset_handler/KittiHandler.hpp>
#include <UnifiedCvo-0.1/utils/Calibration.hpp>
#include <UnifiedCvo-0.1/cvo/CvoGPU.hpp>
#include <UnifiedCvo-0.1/utils/ImageStereo.hpp>
#include <UnifiedCvo-0.1/utils/CvoPointCloud.hpp>

using namespace std;

// class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

// vector<string> readImagePaths(int argc,char **argv,int start){
//     vector<string> paths;
//     for(int i=start;i<argc;i++)
//         paths.push_back(argv[i]);
//     return paths;
// }

// vector<Eigen::Matrix4f> readTransforms(string filename) {
//     cout << "Start reading transform file " << filename << endl;
//     vector<Eigen::Matrix4f> res;
//     ifstream infile(filename);
//     if (infile.is_open()) {
//         float t11, t12 ,t13, t14, t21, t22, t23, t24, t31, t32, t33, t34;
//         while(infile >> t11 >> t12 >> t13 >> t14 >> t21 >> t22 >> t23 >> t24 >> t31 >> t32 >> t33 >> t34) {
//         Eigen::Matrix4f TF = Eigen::Matrix4f::Identity();
//         TF(0,0) = t11;
//         TF(0,1) = t12;
//         TF(0,2) = t13;
//         TF(0,3) = t14;
//         TF(1,0) = t21;
//         TF(1,1) = t22;
//         TF(1,2) = t23;
//         TF(1,3) = t24;
//         TF(2,0) = t31;
//         TF(2,1) = t32;
//         TF(2,2) = t33;
//         TF(2,3) = t34;
//         res.push_back(TF);
//         // std::cout<<"TF: \n"<< t11 << t12 << t13 << t14 << t21 << t22 << t23 << t24 << t31 << t32 << t33 << t34<<std::endl;
//         }
//         if (!infile.eof()) {
//         }
//         infile.close();
//     } else {
//         std::cerr<<" transformation file "<<filename<<" not found!\n";
//     }
//     std::cout<<"the transformation file has been read\n";
//     return res;
// }

// void testLoopClosing(cvo::KittiHandler& kitti, cvo::CvoGPU& cvo_align, cvo::Calibration& calib, vector<Eigen::Matrix4f>& TFs) {
// void testLoopClosing(cvo::KittiHandler& kitti, cvo::CvoGPU& cvo_align, cvo::Calibration& calib) {
//     // data parser
//     kitti.set_start_index(0);
//     int total_iter = kitti.get_total_number();
//     // create CvoLoopClosing instance
//     DBoW3::Vocabulary voc("/media/e/loop_closing_slam/DBow3/orbvoc.dbow3");
//     DBoW3::Database db(voc, false, 0);
//     cvo::CvoLoopClosing clc(&db, &cvo_align, &calib);

//     // Eigen::Matrix4f source_tf = TFs[0];
//     for (int i = 0; i < total_iter; i+=10) {
//         cv::Mat left, right;
//         if (kitti.read_next_stereo(left, right) != 0) {
//             cout << "Finished\n";
//             break;
//         }
//         std::shared_ptr<cvo::ImageStereo> target_raw(new cvo::ImageStereo(left, right));
//         std::shared_ptr<cvo::CvoPointCloud> cur_pc(new cvo::CvoPointCloud(*target_raw, cur_right, calib));
//         cvo::KeyFrame cur_kf(cur_left, *cur_pc);
//         clc.detect_loop(cur_kf);
//         cv::imshow("cur", cur_left);
//         cv::waitKey(10);
//         kitti.next_frame_index();
//     }
//     cout << "... done!\n";
// }


//$bin_file $dataset_dir $param_file $seq $method $transform_file
int main(int argc,char *argv[]) {

    if(!argv[1] || !argv[2]){
        std::cerr << "No options detect\n";
    }
    std::string outFile = argv[2];
    cvo::KittiHandler kitti(argv[1], cvo::KittiHandler::DataType::STEREO);
    // string cvo_param_file(argv[2]);
    // string calib_file = string(argv[1]) + "/cvo_calib.txt";
    // cvo::Calibration calib(calib_file);
    // string sequence(argv[3]);

    // string method(argv[4]);
    // VO results
    // vector<Eigen::Matrix4f> TFs = readTransforms(string(argv[5]));
    // CVO instance
    // cvo::CvoGPU cvo_align(cvo_param_file);
    kitti.set_start_index(0);
    int total_iter = kitti.get_total_number();

    DBoW3::Vocabulary voc("/media/e/loop_closing_slam/thirdparty/DBow3/orbvoc.dbow3");
    DBoW3::Database db(voc, false, 0);
    cvo::CvoLoopClosing clc(&db, total_iter);

    for (int i = 0; i < total_iter; i += 1) {
        cv::Mat left, right;
        if (kitti.read_next_stereo(left, right) != 0) {
            cout << "Finished\n";
            break;
        }
        // std::shared_ptr<cvo::ImageStereo> target_raw(new cvo::ImageStereo(left, right));
        // std::shared_ptr<cvo::CvoPointCloud> cur_pc(new cvo::CvoPointCloud(*target_raw, calib, cvo::CvoPointCloud::CV_ORB));
        // cvo::KeyFrame cur_kf(left, *cur_pc);

        clc.detect_loop(left, kitti.get_current_index());
            
        kitti.next_frame_index();
    }
    clc.print_loop(outFile);
    cout << "... done!\n";
    return 0;
}