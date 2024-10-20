#include <Eigen/Eigen>
#include <iostream>
#include <cstdlib>
#include <unsupported/Eigen/SparseExtra>
#include <Eigen/IterativeLinearSolvers>

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

int main(){
  const char *input_image_path = "assets/256px-Albert_Einstein_Head.jpg";

  int width, height, channels;
  unsigned char *image_data = stbi_load(input_image_path, &width, &height, &channels, 1);

  if (!image_data) {
    std::cerr << "Error: Could not load image " << input_image_path << std::endl;
    return 1;
  }

  // POINT_1
  MatrixXd originalEinsteinMat(height, width);
  for (int i = 0; i < height; i++){
    for (int j = 0; j < width; j++){
      int index = (i * width + j);
      originalEinsteinMat(i, j) = static_cast<double>(image_data[index]);
    }
  }

  MatrixXd task1Mat= (originalEinsteinMat.transpose()) * originalEinsteinMat;
  std::cout << "A(T) * A norm is: " << task1Mat.norm() << std::endl;

  printImage("outputImages/0_testFirstOutput.png", height, width, originalEinsteinMat);

  // POINT_2
  EigenSolver<MatrixXd> eigensolver(task1Mat);
  VectorXcd eigenValues = eigensolver.eigenvalues();
  
  std::cout << "First eigenvalue: " << eigenValues(0) << std::endl;
  std::cout << "Second eigenvalue: " << eigenValues(1) << std::endl;

  // POINT_3
  // SparseMatrix<Double> task3Matrix;
  std::string matrixFileOut("./lis-2.1.6/test/task1Mat.mtx"); // task1Mat is A^T * A
  saveMarket(task1Mat, matrixFileOut);
  // mpirun -n 1 ./eigen1 task1Mat.mtx eigvec_task3.txt hist_task3.txt-e pi -etol 1.e-8
  // e.v. with eigensolver = 1.04582e+09,0
  // e.v with lis = 1.045818e+09

  //POINT_4
  
}