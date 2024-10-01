#include <Eigen/Eigen>
#include <iostream>
#include <cstdlib>

#define STB_IMAGE_IMPLEMENTATION
#include "../external_libraries/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../external_libraries/stb_image_write.h"

int main(){
    const char* input_image_path = "assets/256px-Albert_Einstein_Head.jpg";

    int width, height, channels;
    unsigned char* image_data = stbi_load(input_image_path, &width, &height, &channels, 1);
    
    if (!image_data) {
    std::cerr << "Error: Could not load image " << input_image_path << std::endl;
    return 1;
    }

    Eigen::MatrixXd originalEinstenMat(height,width);
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
            int index = (i * width + j) * channels; 
            originalEinstenMat(i,j) = static_cast<double>(image_data[index]);
        }
    }

    //Point 1
    std::cout << "Image loaded: " << width << "x" << height << " with " << channels << " channels." << std::endl;
    //std::cout << "Matrix of the original image: " << originalEinsteinMat << std::endl;
    std::cout << "Size of the matrix based on the image: " <<originalEinstenMat.size() << std::endl;
    
    //Check if the first images is translated correctly
    const std::string output_image_path = "outputImages/testFirstOutput.png";
    if (stbi_write_png(output_image_path.c_str(), width, height, 1, originalEinstenMat.data(), width) == 0) {
    std::cerr << "Error: Could not save grayscale image" << std::endl;

    return 1;
  }


    stbi_image_free(image_data);

    return 0;
}