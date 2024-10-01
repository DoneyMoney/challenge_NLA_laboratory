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

    std::cout << "Image loaded: " << width << "x" << height << " with " << channels << " channels." << std::endl;
    return 0;
}