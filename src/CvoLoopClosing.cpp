#include "CvoLoopClosing.hpp"
#include <iostream>

namespace cvo {
    CvoLoopClosing::CvoLoopClosing(DBoW3::Database* pDB, int numFrame) :
                                                            db(pDB), 
                                                            frameGap_(10),
                                                            numFrame_(numFrame){
    }

    bool CvoLoopClosing::detect_loop(const cv::Mat& currImg, int currId) {
        // feature detection
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> kp;
        orb->detectAndCompute(currImg, cv::Mat(), kp, descriptors);
        
        // query the db
        DBoW3::QueryResults ret;
        db->query(descriptors, ret, 5);

        // add to db
        db->add(descriptors);

        for (auto r : ret){
          if(currId - r.Id >= frameGap_ && r.Score > 0.055){
            loopsDetect[currId].push_back(r.Id);
            std::cout << "Loop on index: " << currId << "\n";
          }
        }

        if(loopsDetect.empty())
          return false;
        return true;
    }

    void CvoLoopClosing::print_loop(){
      if(loopsDetect.empty()){
        std::cout << "No Loops Detect.\n";
      }

      for (auto v : loopsDetect){
        if (!v.second.empty()){
          std::cout << "Index: " << v.first << " Loops: ";
          for (auto idx : v.second)
             std::cout << idx << " ";
          std:: cout << "\n";
        }
      }
    }
    
} //namespace cvo