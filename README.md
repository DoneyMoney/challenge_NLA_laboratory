## Challanges NLA

# Challenge 1
1) Load the image as an Eigen matrix with size m × n. Each entry in the matrix corresponds to a pixel on the screen and takes a value somewhere between 0 (black) and 255 (white). Report the size of the matrix. 
2) Introduce a noise signal into the loaded image by adding random fluctuations of color ranging between [−50, 50] to each pixel. Export the resulting image in .png and upload it.
3) Reshape the original and noisy images as vectors v and w, respectively. Verify that each vector has m n components. Report here the Euclidean norm of v.
4) Write the convolution operation corresponding to the smoothing kernel Hav2 as a matrix vector multiplication between a matrix A1 having size mn × mn and the image vector. Report the number of non-zero entries in A1.
5) Apply the previous smoothing filter to the noisy image by performing the matrix vector multiplication A1w. Export and upload the resulting image.
6) Write the convolution operation corresponding to the sharpening kernel Hsh2 as a matrix vector multiplication by a matrix A2 having size mn×mn. Report the number of non-zero entries in A2. Is A2 symmetric?
7) Apply the previous sharpening filter to the original image by performing the matrix vector multiplication A2v. Export and upload the resulting image.
8) Export the Eigen matrix A2 and vector w in the .mtx format. Using a suitable iterative solver and preconditioner technique available in the LIS library compute the approximate solution to the linear system A2x = w prescribing a tolerance of 10−9. Report here the iteration count and the final residual.
9) Import the previous approximate solution vector x in Eigen and then convert it into a .png image. Upload the resulting file here.
10) Write the convolution operation corresponding to the detection kernel Hlap as a matrix vector multiplication by a matrix A3 having size mn × mn. Is matrix A3 symmetric?
11) Apply the previous edge detection filter to the original image by performing the matrix vector multiplication A3 v. Export and upload the resulting image.
12) Using a suitable iterative solver available in the Eigen library compute the approximate solution of the linear system (I+A3)y = w, where I denotes the identity matrix, prescribing a tolerance of 10−10. Report here the iteration count and the final residual.
13) Convert the image stored in the vector y into a .png image and upload it.
14) Comment the obtained results.

# Challenge 2
1) Load the image as an Eigen matrix A with size m×n. Each entry in the matrix corresponds to a pixel on the screen and takes a value somewhere between 0 (black) and 255 (white). Compute the matrix product  A^T A  and report the Euclidean norm of  A^T A .
2) Solve the eigenvalue problem  A^T A x = \lambda x  using the proper solver provided by the Eigen library. Report the two largest computed singular values of A.
3) Export matrix  A^T A  in the Matrix Market format and move it to the lis-2.1.6/test folder. Using the proper iterative solver available in the LIS library, compute the largest eigenvalue of  A^T A  up to a tolerance of  10^{-8} . Report the computed eigenvalue. Is the result in agreement with the one obtained in the previous point?
4) Find a shift  \mu \in \mathbb{R}  yielding an acceleration of the previous eigensolver. Report  \mu  and the number of iterations required to achieve a tolerance of  10^{-8}.
5) Using the SVD module of the Eigen library, perform a singular value decomposition of the matrix A. Report the Euclidean norm of the diagonal matrix  \Sigma  of the singular values.
6) Compute the matrices C and D described in (1) assuming  k = 40  and  k = 80 . Report the number of nonzero entries in the matrices C and D.
7) Compute the compressed images as the matrix product  C D^T  (again for  k = 40  and  k = 80 ). Export and upload the resulting images in .png.
8) Using Eigen, create a black and white checkerboard image (as the one depicted below) with height and width equal to 200 pixels. Report the Euclidean norm of the matrix corresponding to the image.
9) Introduce noise into the checkerboard image by adding random fluctuations of color ranging between [−50, 50] to each pixel. Export the resulting image in .png and upload it.
10) Using the SVD module of the Eigen library, perform a singular value decomposition of the matrix corresponding to the noisy image. Report the two largest computed singular values.
11) Starting from the previously computed SVD, create the matrices C and D defined in (1) assuming  k = 5  and  k = 10 . Report the size of the matrices C and D.
12) Compute the compressed images as the matrix product  C D^T  (again for  k = 5  and  k = 10 ). Export and upload the resulting images in .png.
13) Compare the compressed images with the original and noisy images. Comment on the results.
