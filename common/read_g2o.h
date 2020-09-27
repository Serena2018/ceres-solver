#ifndef READ_G2O_H_
#define READ_G2O_H_
#include <iostream>
#include <fstream>
#include <string>

#include <glog/logging.h>

namespace ceres_study{

template <typename Pose, typename Allocator>
bool ReadVertex(std::ifstream* infile,
                std::map<int, Pose, std::less<int>, Allocator>* poses){
    int id;
    Pose pose;
    *infile >> id >> pose;
    if(poses->find(id) != poses->end()){
        LOG(ERROR) << "Duplicate vertex with ID : "<<id;
        return false;
    }
    (*poses)[id] = pose;
    return true;
    }

    template <typename Constraint,typename Allocator>
    void ReadConstraint(std::ifstream* infile,
                        std::vector<Constraint, Allocator>* constraints){
                            Constraint constraint;
                            *infile >> constraint;
                            //std::cout<<"constraint: "<<constraint<<std::endl;
                            constraints->push_back(constraint);
                        }
    //Reads a file in the g2o filename format that describes a pose graph problem
    //The g2o format consists of two entries, Vertices and constraints.
    //In 2D, a vertex is defined as follows:
    //VERTEX_SE2 ID x_meters, y_meters, yaw_radians

    //A constraint is defined as follows:
    //EDGE_SE2 ID_A ID_B A_x_B A_y_B A_yaw_B I_11 I_12 I_13 I_22 I_23 I_33
    //where I_ij is the (i,j)-th entry of the information matrix for the measurement.

    //In 3D, a vertex is defined as folows:
    // VERTEX_SE3: QUAT ID x y z q_x q_y q_z q_w
    //EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 ..... I_16
    template <typename Pose, typename Constraint, typename MapAllocator,
                typename VectorAllocator>
    bool ReadG2oFile(const std::string& filename,
                    std::map<int, Pose, std::less<int>, MapAllocator>* poses,
                    std::vector<Constraint, VectorAllocator>* constraints){
        CHECK(poses != NULL);
        CHECK(constraints !=NULL);
        poses->clear();
        constraints->clear();

        //std::cout<<"filename: "<<filename<<std::endl;

        std::ifstream infile(filename.c_str());
        if(!infile){
            return false;
        }

        std::string data_type;
        while(infile.good()){
            infile >> data_type;
            //std::cout<<"data_type: "<<data_type<<std::endl;
            if(data_type == Pose::name()){
                
                if(!ReadVertex(&infile, poses)){
                    return false;
                }
            }
            else if(data_type == Constraint::name()){
                ReadConstraint(&infile, constraints);
            }
            else{
                LOG(ERROR) << "Unknown data type: "<<data_type;
                return false;
            }
            infile >> std::ws;
        }
        return true;

    }

}//namespace ceres_study

#endif