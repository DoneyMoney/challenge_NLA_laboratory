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
SparseMatrix<double> computeConvMatr(int height, int width, MatrixXd filter){

  SparseMatrix<double> matr(height*width,height*width);
  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(height * width * 9);

  for (int i=0; i < height*width; i++){
    //matr.insert(i,i) = filter(1,1);
    tripletList.push_back(T(i,i,filter(1,1)));
    if(i+4 < height*width){
      tripletList.push_back(T(i,i+1,filter(0,1)));
      tripletList.push_back(T(i,i+2,filter(1,2)));
      tripletList.push_back(T(i,i+3,filter(0,0)));
      tripletList.push_back(T(i,i+4,filter(0,2)));
      /*
      matr.insert(i,i+1) = filter(0,1);
      matr.insert(i,i+2) = filter(1,2);
      matr.insert(i,i+3) = filter(1,0);
      matr.insert(i,i+4) = filter(1,2);
      */
    }else if(i+3 < height*width){
      tripletList.push_back(T(i,i+1,filter(0,1)));
      tripletList.push_back(T(i,i+2,filter(1,2)));
      tripletList.push_back(T(i,i+3,filter(0,0)));
      /*
      matr.insert(i,i+1) = filter(0,1);
      matr.insert(i,i+2) = filter(1,2);
      matr.insert(i,i+3) = filter(1,0);
      */
    }else if(i+2 < height*width){

      tripletList.push_back(T(i,i+1,filter(0,1)));
      tripletList.push_back(T(i,i+2,filter(1,2)));
      /*
      matr.insert(i,i+1) = filter(0,1);
      matr.insert(i,i+2) = filter(1,2);
      */
    }else if(i+1 < height*width){
      
      tripletList.push_back(T(i,i+1,filter(0,1)));
      //matr.insert(i,i+1) = filter(0,1);
    }
    if(i-4 >= 0){
      tripletList.push_back(T(i,i-4,filter(2,2)));
      tripletList.push_back(T(i,i-3,filter(2,0)));
      tripletList.push_back(T(i,i-2,filter(1,0)));
      tripletList.push_back(T(i,i-1,filter(2,1)));
      /*
      matr.insert(i,i-4) = filter(0,0);
      matr.insert(i,i-3) = filter(2,0);
      matr.insert(i,i-2) = filter(1,0);
      matr.insert(i,i-1) = filter(2,1);
      */
    }else if(i-3 >= 0){
      tripletList.push_back(T(i,i-3,filter(2,0)));
      tripletList.push_back(T(i,i-2,filter(1,0)));
      tripletList.push_back(T(i,i-1,filter(2,1)));
      /*
      matr.insert(i,i-3) = filter(1,2);
      matr.insert(i,i-2) = filter(1,0);
      matr.insert(i,i-1) = filter(2,1);
      */
    }else if(i-2 >= 0){
      tripletList.push_back(T(i,i-2,filter(1,0)));
      tripletList.push_back(T(i,i-1,filter(2,1)));
      /*
      matr.insert(i,i-2) = filter(1,0);
      matr.insert(i,i-1) = filter(1,2);
      */
    }else if(i-1 >= 0){
      tripletList.push_back(T(i,i-1,filter(2,1)));
      //matr.insert(i,i-1) = filter(1,2);
    }
  }

  std::cout << "Number of non-zero elements from A1 is: " << tripletList.size() << std::endl;
  matr.setFromTriplets(tripletList.begin(), tripletList.end());
  return matr;
}



int main(){
  
  const char* input_image_path = "assets/256px-Albert_Einstein_Head.jpg";

  int width, height, channels;
  unsigned char* image_data = stbi_load(input_image_path, &width, &height, &channels, 1);
  
  if (!image_data) {
    std::cerr << "Error: Could not load image " << input_image_path << std::endl;
    return 1;
  }

  //Point 1
  Eigen::MatrixXd originalEinsteinMat(height,width);
  for(int i=0;i<height;i++){
      for(int j=0;j<width;j++){
          int index = (i * width + j); 
          originalEinsteinMat(i,j) = static_cast<double>(image_data[index]);
      }
  }

  printImage("outputImages/testFirstOutput.png",height,width,originalEinsteinMat);

  std::cout << "Image loaded: " << width << "x" << height << " with " << channels << " channels." << std::endl;
  stbi_image_free(image_data);

  //Point2
  Eigen::MatrixXd noise = Eigen::MatrixXd::Random(height,width);

  Eigen::MatrixXd noisy_image = originalEinsteinMat;
  for(int i = 0; i<height;i++){
    for(int j=0;j<width;j++){
      noisy_image(i,j) = noisy_image(i,j) + (noise(i,j) * 50);
      noisy_image(i,j) = noisy_image(i,j) > 255.0 ? 255.0 : noisy_image(i,j);
      noisy_image(i,j) = noisy_image(i,j) < 0 ? 0 : noisy_image(i,j);
    }
  }
  printImage("outputImages/noisyImage.png",height,width,noisy_image);

  //Point3
  Eigen::VectorXd vector_V(originalEinsteinMat.size());
  int index = 0;
  for(int i = 0; i < originalEinsteinMat.rows(); i++) {
      for(int j = 0; j < originalEinsteinMat.cols(); j++) {
          vector_V(index) = originalEinsteinMat(i, j);
          index++;
      }
  }
  if(vector_V.size() == height * width){
    std::cout << "The sizes of the matrix and the vector v (original One) corresponds" << std::endl;
  }
  else{
    std::cout << "The sizes of the matrix and the vector v DON'T corresponds" << std::endl;
  }

  Eigen::VectorXd vector_W(noisy_image.size());
  index = 0;
  for(int i = 0; i < noisy_image.rows(); i++) {
      for(int j = 0; j < noisy_image.cols(); j++) {
          vector_W(index) = noisy_image(i, j);
          index++;
      }
  }
  if(vector_W.size() == height * width){
    std::cout << "The sizes of the matrix and the vector w (noisy one) corresponds" << std::endl;
  }
  else{
    std::cout << "The sizes of the matrix and the vector w DON'T corresponds" << std::endl;
  }

  std::cout << "The vector v norm is: " << vector_V.norm() <<std::endl;
  
  //Point4
  Eigen::MatrixXd smoothedMatrix = Eigen::MatrixXd::Zero(height, width);
  Eigen::MatrixXd Hav2 = Eigen::MatrixXd::Constant(3,3,0.1111111);
  
  //calculate convmatrixA1
  double mTimesn = height*width;
  SparseMatrix<double> convMatrixA1(height * width, height * width); 

  convMatrixA1 = computeConvMatr(height,width,Hav2);

  std::cout << "Sparse matrix: "<<std::endl << convMatrixA1.topLeftCorner(15,15) <<std::endl ;

  smoothedMatrix = convMatrixA1 * vector_V;
  

  return 0;
}