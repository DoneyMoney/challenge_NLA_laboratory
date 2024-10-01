#include <Eigen/Eigen>
#include <iostream>
#include <cstdlib>

#define STB_IMAGE_IMPLEMENTATION
#include "../external_libraries/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../external_libraries/stb_image_write.h"

using namespace Eigen;

int main(){
    const char* input_image_path = "assets/256px-Albert_Einstein_Head.jpg";

    int width, height, channels;
    unsigned char* image_data = stbi_load(input_image_path, &width, &height, &channels, 1);
    
    if (!image_data) {
    std::cerr << "Error: Could not load image " << input_image_path << std::endl;
    return 1;
    }

    Eigen::MatrixXd originalEinsteinMat(height,width);
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
            int index = (i * width + j) * channels; 
            originalEinsteinMat(i,j) = static_cast<double>(image_data[index]) / 255.0;
        }
    }

    //Point 1
    std::cout << "Image loaded: " << width << "x" << height << " with " << channels << " channels." << std::endl;
    //std::cout << "Matrix of the original image: " << originalEinsteinMat << std::endl;
    std::cout << "Size of the matrix based on the image: " <<originalEinsteinMat.size() << std::endl;
    
    Matrix<unsigned char, Dynamic, Dynamic, RowMajor> output_image(width, height);
  // Use Eigen's unaryExpr to map the grayscale values (0.0 to 1.0) to 0 to 255
    output_image = originalEinsteinMat.unaryExpr([](double val) -> unsigned char {
    return static_cast<unsigned char>(val * 255.0);
    });

    //Check if the first images is translated correctly
    const std::string output_image_path = "testFirstOutput.png";

    if (stbi_write_png(output_image_path.c_str(), width, height, 1, output_image.data(), width) == 0) {
    std::cerr << "Error: Could not save grayscale image" << std::endl;

    return 1;
  }



    std::cout << "Images saved to " << output_image_path << " and " << output_image_path << std::endl;
    stbi_image_free(image_data);

    return 0;
}