#include <Eigen/Eigen>
#include <iostream>
#include <cstdlib>

#define STB_IMAGE_IMPLEMENTATION
#include "../external_libraries/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../external_libraries/stb_image_write.h"

using namespace Eigen;


int printImage(std::string path, int h , int w , Eigen::MatrixXd mat){
  Matrix<unsigned char, Dynamic, Dynamic, RowMajor> out_image(w,h);
  // Use Eigen's unaryExpr make the conversion
  out_image = mat.unaryExpr([](double val) -> unsigned char {
    return static_cast<unsigned char>(val);
  });

  if (stbi_write_png(path.c_str(), w,h, 1, out_image.data(), w) == 0) {
    std::cerr << "Error: Could not save grayscale image" << std::endl;

    return 1;
  }

  return 0;
}

//We are assuming a 3x3 filter matrix 
Eigen::MatrixXd computeConvMatr(int height, int width, MatrixXd filter){

  Eigen::MatrixXd matr = Eigen::MatrixXd::Zero(height*width, height*width);
  for (int i=0; i < height*width; i++){
    matr(i,i) = filter(1,1);
    if(i+4 < height*width){
      matr(i,i+1) = filter(0,1);
      matr(i,i+2) = filter(1,2);
      matr(i,i+3) = filter(1,0);
      matr(i,i+4) = filter(1,2);
    }else if(i+3 < height*width){
      matr(i,i+1) = filter(0,1);
      matr(i,i+2) = filter(1,2);
      matr(i,i+3) = filter(1,0);
    }else if(i+2 < height*width){
      matr(i,i+1) = filter(0,1);
      matr(i,i+2) = filter(1,2);
    }else if(i+1 < height*width){
      matr(i,i+1) = filter(0,1);
    }

    if(i-4 >= 0){
      matr(i,i-4) = filter(0,0);
      matr(i,i-3) = filter(2,0);
      matr(i,i-2) = filter(1,0);
      matr(i,i-1) = filter(2,1);
    }else if(i-3 >= 0){
      matr(i,i-3) = filter(1,2);
      matr(i,i-2) = filter(1,0);
      matr(i,i-1) = filter(2,1);
    }else if(i-2 >= 0){
      matr(i,i-2) = filter(1,0);
      matr(i,i-1) = filter(1,2);
    }else if(i-1 >= 0){
      matr(i,i-1) = filter(1,2);
    }
  }
  return matr;
}

int main(){
    //Point4
    int height = 256, width = 341;

    Eigen::MatrixXd smoothedMatrix = Eigen::MatrixXd::Zero(height, width);
    Eigen::MatrixXd Hav2(3,3);
    Hav2 << 1/9, 1/9, 1/9,
            1/9, 1/9, 1/9,
            1/9, 1/9, 1/9;
    
    //calculate convmatrixA1
    double mTimesn = height*width;
    SparseMatrix<double> convMatrixA1(height * width, height * width); //Here is the problem!!!
    
    //convMatrixA1 = computeConvMatr(height,width,Hav2);
    
    //smoothedMatrix = convMatrixA1 * vector_V;
    int numOfNonZeroes = 0;
    for(int i = 0; i < smoothedMatrix.rows(); i++) {
        for(int j = 0; j < smoothedMatrix.cols(); j++) {
            if(smoothedMatrix(i,j) != 0){
                numOfNonZeroes++;
            }
        }
    }
    std::cout << "Numer of non-zeroes from the A1*v matrix: " << numOfNonZeroes << " Shoulbe the the order of: " <<height*width <<std::endl;
    
}