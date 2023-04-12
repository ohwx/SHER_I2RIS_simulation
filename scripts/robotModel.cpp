# include <iostream>
# include <eigen3/Eigen/Dense>
# include <cmath>

void ComputeFwdKinsTip(Eigen::VectorXf & joints,
                                   Eigen::Vector3f & tipXYZ,
                                   Eigen::Affine3f & FTip) 
{

    Eigen::Vector3f tip(0,0,0);
    Eigen::Vector3f& tipXYZ = tip;
    double d = 0;
    tipXYZ[0] = 0;
    tipXYZ[1] = 0;
    tipXYZ[2] = 0;
    double lx = tipXYZ[0]; // + ToolOffset[0];
    double ly = tipXYZ[1]; // + ToolOffset[1];
    double lz = tipXYZ[2]; // + ToolOffset[2];
    Eigen::VectorXf joints(7);
    joints << 0,0,0,0,0,0,0;
    double q0 = joints[0];
    double q1 = joints[1];
    double q2 = joints[2];
    double q3 = joints[3];
    double q4 = joints[4];
    FTip = Eigen::Affine3f::Identity();
    double c3 = cos(q3);
    double s3 = sin(q3);
    double c4 = cos(q4);
    double s4 = sin(q4);
    Eigen::AngleAxisf RTip;
    RTip =  Eigen::AngleAxisf(q3, Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(q4,  Eigen::Vector3f::UnitX());
    FTip.translation().x() = q0 + lx*c3 + d*s3 + lz*s3*c4 + ly*s3*s4;
    FTip.translation().y() = q1 + ly*c4 - lz*s4;
    FTip.translation().z() = q2 + d*c3 - lx*s3 + lz*c3*c4 + ly*c3*s4;
    FTip.linear() = RTip.toRotationMatrix();


}

void ComputeFwdKinsTipSnake(Eigen::VectorXf & joints,
                                   Eigen::Affine3f & FTipSnake) const
{
    double q5 = joints[5];
    double q6 = joints[6];
    int n = 13;
    FTipSnake = Eigen::Affine3f::Identity();

    // double check phi1 and phi2 when n not equal to 13
    double phi1 = q5/(n/2);
    double phi2 = q6/(n/2);
    std::cout << phi2 << std::endl;
    Eigen::AngleAxisf rotXThetai = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotXNegThetai = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot;
    Eigen::Affine3f FSnakeCurR;
    Eigen::Affine3f FSnakeCurP;
    for (int i=0; i < n; i++){
        FSnakeCurR = Eigen::Affine3f::Identity();
        FSnakeCurP = Eigen::Affine3f::Identity();
        if (i==0 || i==12){
            FSnakeCurP.translation().x() = 0.45; //mm
        }else{
            FSnakeCurP.translation().x() = 0.15; //mm
        }

        Eigen::AngleAxisf rotYPhii;
        if (i%2 == 0){
            rotYPhii = Eigen::AngleAxisf(phi1, Eigen::Vector3f::UnitY());;
        }else{
            rotYPhii = Eigen::AngleAxisf(phi2, Eigen::Vector3f::UnitY());;
        }
        rot = rotXThetai.toRotationMatrix()*rotYPhii.toRotationMatrix()*rotXNegThetai.toRotationMatrix();
        FSnakeCurR.linear() = rot.toRotationMatrix();;
        FTipSnake = FTipSnake*FSnakeCurR*FSnakeCurP;
    }
    std::cout << FTipSnake.matrix() << std::endl;
  }
