#ifndef depthcal_h_
#define depthcal_h_


void calculate_disparity_jump(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity, int block_size, int jump_factor) {
    int half_block = block_size / 2;

    // Initialize disparity map to zero
    memset(disparity, 0, width * height * sizeof(uint8_t));

    for (int y = half_block; y < height - half_block; y++) {
        for (int x = half_block; x < width - half_block; x++) {
            int min_ssd = INT_MAX;
            int best_disparity = 0;

            // Loop through disparities with the jump factor
            for (int d = 0; d < max_disparity; d += jump_factor) {
                int ssd = 0;

                for (int v = -half_block; v <= half_block; v++) {
                    for (int u = -half_block; u <= half_block; u++) {
                        int left_pixel = imgL[(y + v) * width + (x + u)];
                        int right_pixel = (x + u - d >= 0) ? imgR[(y + v) * width + (x + u - d)] : 0;
                        int diff = left_pixel - right_pixel;
                        ssd += diff * diff;
                    }
                }

                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                }
            }

            // Only update disparity map if the best disparity is within a close range
            if (best_disparity > 0) {
                disparity[y * width + x] = (uint8_t)(best_disparity * 255 / max_disparity);
            }
        }
    }
}

// General
void calculate_disparity_block(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity, int block_size) {
    int half_block = block_size / 2;

    // Initialize disparity map to zero
    memset(disparity, 0, width * height * sizeof(uint8_t));

    for (int y = half_block; y < height - half_block; y++) {
        for (int x = half_block; x < width - half_block; x++) {
            int min_ssd = INT_MAX;
            int best_disparity = 0;

            for (int d = 0; d < max_disparity; d++) {
                int ssd = 0;

                for (int v = -half_block; v <= half_block; v++) {
                    for (int u = -half_block; u <= half_block; u++) {
                        int left_pixel = imgL[(y + v) * width + (x + u)];
                        int right_pixel = (x + u - d >= 0) ? imgR[(y + v) * width + (x + u - d)] : 0;
                        int diff = left_pixel - right_pixel;
                        ssd += diff * diff;
                    }
                }

                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                }
            }

            disparity[y * width + x] = (uint8_t)(best_disparity * 255 / max_disparity);
        }
    }
}


void convert_disparity_to_depth(uint8_t* disparity, float* depth, int width, int height, float fx, float baseline, float units) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int disp = disparity[y * width + x];
            if (disp > 0) {
                depth[y * width + x] = (fx * baseline) / (units * disp);
            } else {
                depth[y * width + x] = 0.0f;  // Handle zero disparity case
            }
        }
    }
}

void calculate_disparity_v2(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity, int block_size, float fx, float baseline, float units) {
    // Initialize disparity map to zero
    memset(disparity, 0, width * height * sizeof(uint8_t));

    for (int y = block_size; y < height - block_size - 1; y++) {
        for (int x = block_size + max_disparity; x < width - block_size - 1; x++) {
            int min_ssd = INT_MAX;
            int best_disparity = 0;

            // Compare blocks between left and right images
            for (int d = 0; d < max_disparity; d++) {
                int ssd = 0;

                for (int i = -block_size; i <= block_size; i++) {
                    for (int j = -block_size; j <= block_size; j++) {
                        int left_pixel = imgL[(y + i) * width + (x + j)];
                        int right_pixel = (x - d + j >= 0) ? imgR[(y + i) * width + (x - d + j)] : 0;

                        int diff = left_pixel - right_pixel;
                        ssd += diff * diff;  // Sum of squared differences for block
                    }
                }

                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                }
            }

            // Store the best disparity
            disparity[y * width + x] = (uint8_t)(best_disparity * 255 / max_disparity);
        }
    }
}


void calculate_disparity(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity) {
    // Initialize disparity map to zero
    memset(disparity, 0, width * height * sizeof(uint8_t));

    for (int y = 0; y < height; y++) {
        for (int x = max_disparity-1; x < width-(max_disparity/2); x++) {
            int min_ssd = INT_MAX;
            int best_disparity = 0;

            // Compare pixels between left and right images
            for (int d = 0; d < max_disparity; d++) { 
                int ssd = 0;

                int left_pixel = imgL[y * width + x];
                int right_pixel = (x - d >= 0) ? imgR[y * width + (x - d)] : 0;

                int diff = left_pixel - right_pixel;
                ssd = diff * diff;  // Sum of squared differences for single pixel

                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                
            }

            // Store the best disparity scaled to 255
            disparity[y * width + x] = (uint8_t)(best_disparity * 255 / max_disparity);
        }
    }
}
}

void calculate_disparity_half(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity) {
    // Initialize disparity map to zero
    memset(disparity, 0, width * height * sizeof(uint8_t));

    // Start processing from the middle of the images
    int half_width = width / 2;

    for (int y = 0; y < height; y++) {
        for (int x = half_width; x < width - max_disparity; x++) {
            int min_ssd = INT_MAX;
            int best_disparity = 0;

            // Compare pixels between the right half of the left frame and the left half of the right frame
            for (int d = 0; d < max_disparity; d++) {
                int ssd = 0;

                // Left frame: Right half pixels
                int left_pixel = imgL[y * width + x];

                // Right frame: Left half pixels, shifted by disparity d
                int right_pixel = (x - half_width - d >= 0) ? imgR[y * width + (x - half_width - d)] : 0;

                // Calculate the sum of squared differences for single pixel
                int diff = left_pixel - right_pixel;
                ssd = diff * diff;

                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                }
            }

            // Store the best disparity scaled to 255
            disparity[y * width + x] = (uint8_t)(best_disparity * 255 / max_disparity);
        }
    }
}

// Function to calculate binary descriptors
void compute_binary_descriptor(uint8_t* img, int width, int x, int y, int block_size, uint64_t* descriptor) {
    int half_block = block_size / 2;
    int threshold = 128; // Threshold value for binary encoding

    *descriptor = 0;
    int bit_pos = 0;

    for (int by = -half_block; by <= half_block; by++) {
        for (int bx = -half_block; bx <= half_block; bx++) {
            int pixel_x = x + bx;
            int pixel_y = y + by;

            // Boundary check
            if (pixel_x >= 0 && pixel_x < width && pixel_y >= 0 && pixel_y < width) {
                int pixel = img[pixel_y * width + pixel_x];
                uint64_t bit = (pixel > threshold) ? 1 : 0;
                *descriptor |= (bit << bit_pos);
            } else {
                // If outside boundaries, treat as zero (or handle as needed)
                *descriptor |= (0 << bit_pos);
            }
            bit_pos++;
        }
    }
}

// Function to calculate Hamming distance between two descriptors
int hamming_distance(uint64_t d1, uint64_t d2) {
    uint64_t x = d1 ^ d2;
    int dist = 0;
    while (x) {
        dist += x & 1;
        x >>= 1;
    }
    return dist;
}

// Function to calculate disparity using block matching
void calculate_disparity_block_half_ORB(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity, int block_size) {
    // Initialize disparity map to zero
    memset(disparity, 0, width * height * sizeof(uint8_t));

    int half_width = width / 2;
    int half_block = block_size / 2;

    for (int y = half_block; y < height - half_block; y++) {
        for (int x = half_width + half_block; x < width - max_disparity - half_block; x++) {
            int min_hamming = INT_MAX;
            int best_disparity = 0;

            // Compute binary descriptor for the current block in the left image
            uint64_t left_descriptor;
            compute_binary_descriptor(imgL, width, x, y, block_size, &left_descriptor);

            // Compare blocks between the right half of the left frame and the left half of the right frame
            for (int d = 0; d < max_disparity; d++) {
                uint64_t right_descriptor;
                compute_binary_descriptor(imgR, width, x - half_width - d, y, block_size, &right_descriptor);

                int hamming = hamming_distance(left_descriptor, right_descriptor);

                if (hamming < min_hamming) {
                    min_hamming = hamming;
                    best_disparity = d;
                }
            }

            // Store the best disparity scaled to 255
            disparity[y * width + x] = (uint8_t)(best_disparity * 255 / max_disparity);
        }
    }
}

void calculate_disparity_block_half(uint8_t* imgL, uint8_t* imgR, uint8_t* disparity, int width, int height, int max_disparity, int block_size) {
    // Initialize disparity map to zero
    memset(disparity, 0, width * height * sizeof(uint8_t));

    int half_width = width / 3;
    int half_block = block_size / 2;

    for (int y = half_block; y < height - half_block; y++) {
        for (int x = half_width + half_block; x < width - max_disparity - half_block; x++) {
            int min_ssd = INT_MAX;
            int best_disparity = 0;

            // Compare blocks between right half of left frame and left half of right frame
            for (int d = 0; d < max_disparity; d++) {
                int ssd = 0;

                // Iterate over block of pixels around the current pixel
                for (int by = -half_block; by <= half_block; by++) {
                    for (int bx = -half_block; bx <= half_block; bx++) {
                        int left_pixel = imgL[(y + by) * width + (x + bx)];
                        int right_pixel = (x - half_width - d + bx >= 0) ? imgR[(y + by) * width + (x - half_width - d + bx)] : 0;

                        int diff = left_pixel - right_pixel;
                        ssd += diff * diff;  // Accumulate SSD over the block
                    }
                }

                if (ssd < min_ssd) {
                    min_ssd = ssd;
                    best_disparity = d;
                }
            }

            // Store the best disparity scaled to 255
            disparity[y * width + x] = (uint8_t)(best_disparity * 255 / max_disparity);
        }
    }
}





#endif