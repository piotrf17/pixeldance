#include "block_image.h"

#include <cassert>

#include <GL/gl.h>

namespace pixeldance {

BlockImage::BlockImage(const cv::Mat& original, int rows, int cols)
    : original_(original),
      rows_(rows),
      cols_(cols) {
  Build();
}

void BlockImage::Draw(int h) const {
  glBegin(GL_QUADS);
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols_; ++j) {
      const Color& c = blocks_[cols_ * i + j];
      glColor3ub(c.r, c.g, c.b);
      glVertex2f(h * j, h * (rows_ - i - 1));
      glVertex2f(h * (j + 1), h * (rows_ - i - 1));
      glVertex2f(h * (j + 1), h * (rows_ - i));
      glVertex2f(h * j, h * (rows_ - i));
    }
  }
  glEnd();
}

void BlockImage::Build() {
  int s = original_.rows / rows_;
  assert(s * rows_ == original_.rows);
  assert(s * cols_ == original_.cols);
  blocks_.resize(rows_ * cols_);
  const uint8_t* pixels = (const uint8_t*)original_.data;
  const int cn = original_.channels();
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols_; ++j) {
      float r = 0, g = 0, b = 0;
      for (int ii = i * s; ii < (i + 1) * s; ++ii) {
        for (int jj = j * s; jj < (j + 1) * s; ++jj) {
          b += pixels[(ii * original_.cols + jj) * cn + 0];
          g += pixels[(ii * original_.cols + jj) * cn + 1];
          r += pixels[(ii * original_.cols + jj) * cn + 2];
        }
      }
      blocks_[cols_ * i + j].r = static_cast<uint8_t>(r / (s * s));
      blocks_[cols_ * i + j].g = static_cast<uint8_t>(g / (s * s));
      blocks_[cols_ * i + j].b = static_cast<uint8_t>(b / (s * s));        
    }
  }
}

}  // namespace pixeldance
