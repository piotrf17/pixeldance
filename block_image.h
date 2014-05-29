// A BlockImage represents an image that has been "blockified" into a lower
// resolution image made of a few blocks, each with a color that is the average
// of the colors of the original image.
#ifndef PIXELDANCE_BLOCK_IMAGE_H_
#define PIXELDANCE_BLOCK_IMAGE_H_

#include <cstdint>
#include <vector>

#include <opencv2/core/core.hpp>

namespace cv {
class Mat;
}  // namespace cv

namespace pixeldance {

struct Color {
  Color() : r(0), g(0), b(0), a(255) {}
  uint8_t r, g, b, a;
};

class BlockImage {
 public:
  // Convert the given original image into a block.  Currently requires
  // that the original image has size n * rows x n * cols, where n is an
  // integer.
  BlockImage(const cv::Mat& original, int rows, int cols);

  // Draw the block image with scale factor h (size of each block in pixels).
  void Draw(int h) const;

  // Accessors.
  std::vector<Color> blocks() const { return blocks_; }
  int rows() const { return rows_; }
  int cols() const { return cols_; }

 private:
  void Build();
  
  cv::Mat original_;
  int rows_, cols_;
  std::vector<Color> blocks_;
};
  
}  // namespace pixeldance

#endif  // PIXELDANCE_BLOCK_IMAGE_H_
