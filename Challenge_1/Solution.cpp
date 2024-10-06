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

//We are assuming a 3x3 filter matrix 
SparseMatrix<double> computeConvMatr(int height, int width, MatrixXd filter){

  SparseMatrix<double> matr(height*width, height*width);
  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(height * width * 9);
  /*
  for (int i=0; i < height*width; i++){
    tripletList.push_back(T(i,i,filter(1,1)));
    if(i+4 < height*width){
      tripletList.push_back(T(i,i+1,filter(1,2)));
      tripletList.push_back(T(i,i+2,filter(0,1)));
      tripletList.push_back(T(i,i+3,filter(0,0)));
      tripletList.push_back(T(i,i+4,filter(0,2)));
    }else if(i+3 < height*width){
      tripletList.push_back(T(i,i+1,filter(1,2)));
      tripletList.push_back(T(i,i+2,filter(0,1)));
      tripletList.push_back(T(i,i+3,filter(0,0)));
    }else if(i+2 < height*width){
      tripletList.push_back(T(i,i+1,filter(1,2)));
      tripletList.push_back(T(i,i+2,filter(0,1)));
    }else if(i+1 < height*width){
      tripletList.push_back(T(i,i+1,filter(1,2)));
    }
    if(i-4 >= 0){
      tripletList.push_back(T(i,i-4,filter(2,2)));
      tripletList.push_back(T(i,i-3,filter(2,0)));
      tripletList.push_back(T(i,i-2,filter(2,1)));
      tripletList.push_back(T(i,i-1,filter(1,0)));
    }else if(i-3 >= 0){
      tripletList.push_back(T(i,i-3,filter(2,0)));
      tripletList.push_back(T(i,i-2,filter(2,1)));
      tripletList.push_back(T(i,i-1,filter(1,0)));
    }else if(i-2 >= 0){
      tripletList.push_back(T(i,i-2,filter(2,1)));
      tripletList.push_back(T(i,i-1,filter(1,0)));
    }else if(i-1 >= 0){
      tripletList.push_back(T(i,i-1,filter(1,0)));
    }
  }
  */
  int max_col = height*width;
  for (int i=0; i < height*width; i++){
    tripletList.push_back(T(i,i,filter(1,1)));
    int a=0, b=0;
    for(int j = -4; j<5; j++){
      if(i+j >= 0 && i+j<max_col && j!=0){
        tripletList.push_back(T(i, i+j, filter(a,b)));
      }
      b++;
      if(b>=3){
        b = 0;
        a++;
      }
    }
  }
  std::cout << "Number of non-zero elements from matrix is: " << tripletList.size() << std::endl;
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

  //POINT_1
  Eigen::MatrixXd originalEinsteinMat(height,width);
  for(int i=0;i<height;i++){
      for(int j=0;j<width;j++){
          int index = (i * width + j); 
          originalEinsteinMat(i,j) = static_cast<double>(image_data[index]);
      }
  }

  printImage("outputImages/0_testFirstOutput.png", height, width, originalEinsteinMat);

  std::cout << "Image loaded: " << width << "x" << height << " with " << channels << " channels." << std::endl;
  stbi_image_free(image_data);

  //POINT_2 - noise_matrix contains the value [-50; 50]
  // noysi_image is the matrix which represents the image with noise
  Eigen::MatrixXd noise_matrix = Eigen::MatrixXd::Random(height,width);
  Eigen::MatrixXd noise_image = originalEinsteinMat;

  for(int i = 0; i<height;i++){
    for(int j=0;j<width;j++){
      noise_image(i,j) = noise_image(i,j) + (noise_matrix(i,j) * 50);
      noise_image(i,j) = noise_image(i,j) > 255.0 ? 255.0 : noise_image(i,j);
      noise_image(i,j) = noise_image(i,j) < 0 ? 0 : noise_image(i,j);
    }
  }
  printImage("outputImages/2_noisyImage.png", height, width, noise_image);

  // POINT_3 - v and w vectors with m*n components. And norm of 'v' vector
  // v = vector from original image
  // w = vector from noise image
  Eigen::VectorXd vector_V(originalEinsteinMat.size());
  int index = 0;
  int row = originalEinsteinMat.rows();
  int col = originalEinsteinMat.cols();
  for(int i = 0; i < row; i++) {
      for(int j = 0; j < col; j++) {
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

  Eigen::VectorXd vector_W(noise_image.size());
  index = 0;
  for(int i = 0; i < noise_image.rows(); i++) {
      for(int j = 0; j < noise_image.cols(); j++) {
          vector_W(index) = noise_image(i, j);
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
  
  //POINT_4
  Eigen::MatrixXd smoothedMatrix = Eigen::MatrixXd::Zero(height, width);
  double const oneOfNine = 1.0 / 9.0;
  Eigen::MatrixXd Hav2 = Eigen::MatrixXd::Constant(3, 3, oneOfNine);
  
  //calculate convMatrixA1
  SparseMatrix<double> convMatrixA1(height * width, height * width); 
  convMatrixA1 = computeConvMatr(height, width, Hav2);

  SparseMatrix<double> convMatrixA1_transpose = convMatrixA1.transpose();
  if (convMatrixA1.isApprox(convMatrixA1_transpose)) {
    std::cout << "convMatrixA1 is symmetric." << std::endl;
  } else {
    std::cout << "convMatrixA1 is not symmetric." << std::endl;
  }
  smoothedMatrix = convMatrixA1 * vector_V;
  
  //POINT_5
  Eigen::MatrixXd smoothedNoisyMatrix = Eigen::MatrixXd::Zero(height, width);
  smoothedNoisyMatrix = convMatrixA1 * vector_W;

  printImage("outputImages/5_smoothedNoisyImage.png", height, width, smoothedNoisyMatrix);

  //POINT_6
  Eigen::MatrixXd Hsh2(3, 3);
  Hsh2 << 0, -3, 0,
          -1, 9, -3,
          0, -1, 0;

  SparseMatrix<double> convMatrixA2(height * width, height * width); 
  convMatrixA2 = computeConvMatr(height, width, Hsh2);

  // Ensure convMatrixA2 is symmetric
  //Il metodo isApprox() di Eigen verifica se due matrici sono approssimativamente uguali,
  //tenendo conto di eventuali errori numerici. È consigliato per matrici con numeri a virgola mobile.
  //Il controllo elemento per elemento killava il processo
  SparseMatrix<double> convMatrixA2_transpose = convMatrixA2.transpose();
  if (convMatrixA2.isApprox(convMatrixA2_transpose)) {
    std::cout << "convMatrixA2 is symmetric." << std::endl;
  } else {
    std::cout << "convMatrixA2 is not symmetric." << std::endl;
  }

  //POINT_7
  Eigen::MatrixXd sharpenedMatrix = Eigen::MatrixXd::Zero(height, width);
  sharpenedMatrix = convMatrixA2 * vector_V;

  printImage("outputImages/7_sharpenedImage.png",height, width, sharpenedMatrix);

  //POINT_8
  std::string matrixA2FileOut("./Point8Files/A2matrix.mtx");
  std::string vectorWFileOut("./Point8Files/Wvector.mtx");

  Eigen::saveMarket(convMatrixA2, matrixA2FileOut);
  Eigen::saveMarket(vector_W, vectorWFileOut);

  //POINT_9
  VectorXd solutionX(height*width);
  loadMarketVector(solutionX,"/Point8Files/sol.mtx");
  Eigen::MatrixXd solutionXMatrix(height,width);
  
  for(int i=0;i<height;i++){
      for(int j=0;j<width;j++){
          int index = (i * width + j); 
          solutionXMatrix(i,j) = static_cast<double>(solutionX[index]);
      }
  }
  printImage("outputImages/9_Image.png",height, width, solutionXMatrix);

  //POINT_10
  Eigen::MatrixXd Hlap(3,3);
  Hlap << 0, -1, 0,
          -1, 4, -1,
          0, -1, 0;

  SparseMatrix<double> convMatrixA3(height * width, height * width); 
  convMatrixA3 = computeConvMatr(height,width,Hlap);

  SparseMatrix<double> convMatrixA3_transpose = convMatrixA3.transpose();
  if (convMatrixA3.isApprox(convMatrixA3_transpose)) {
    std::cout << "convMatrixA3 is symmetric." << std::endl;
  } else {
    std::cout << "convMatrixA3 is not symmetric." << std::endl;
  }

  //POINT_11
  Eigen::MatrixXd matrixWithEdgeDetection(height,width);
  matrixWithEdgeDetection = convMatrixA3 * vector_V;
  printImage("outputImages/11_imageWithEdgeDetection.png",height,width,matrixWithEdgeDetection);

  //POINT_12
  SparseMatrix<double> identityMatrix(height * width, height * width);
  identityMatrix.reserve(height*width);
  for(int i=0;i < height*width;i++) identityMatrix.insert(i,i) = 1.0;

  SparseMatrix<double> identityPlusA3(height * width, height * width);
  identityPlusA3 = identityMatrix + convMatrixA3;
  // Solving 
  BiCGSTAB<Eigen::SparseMatrix<double> > solver(identityPlusA3);   // factorization 
  solver.setTolerance(1e-10);
  solver.compute(identityPlusA3);
  if(solver.info()!=Success) {                                 
      std::cout << "cannot factorize the matrix" << std::endl;          
      return 0;
  }
  
  VectorXd solutionY = solver.solve(vector_W);                   
  std::cout << "The system resolution for Point 12 ended with " <<solver.iterations()
   << " iteration and with a residual of: " << solver.error() <<std::endl;
  
  //POINT_13
  Eigen::MatrixXd solutionYMatrix(height,width);
  for(int i=0;i<height;i++){
      for(int j=0;j<width;j++){
          int index = (i * width + j); 
          solutionYMatrix(i,j) = static_cast<double>(solutionY[index]);
      }
  }
  printImage("outputImages/13_Image.png",height,width,solutionYMatrix);


  return 0;
}