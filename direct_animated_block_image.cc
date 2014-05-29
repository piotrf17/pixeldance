#include "block_image.h"
#include "direct_animated_block_image.h"

#include <GL/gl.h>

namespace pixeldance {

DirectAnimatedBlockImage::Block::Block(const Color& start_color,
                                       const Color& finish_color,
                                       double start_x, double start_y,
                                       double finish_x, double finish_y)
    : start_color_(start_color),
      finish_color_(finish_color),
      start_x_(start_x), start_y_(start_y),
      finish_x_(finish_x), finish_y_(finish_y) {
}

Color DirectAnimatedBlockImage::Block::GetColor(double t) const {
  Color c;
  c.r = round((1 - t) * start_color_.r + t * finish_color_.r);
  c.g = round((1 - t) * start_color_.g + t * finish_color_.g);
  c.b = round((1 - t) * start_color_.b + t * finish_color_.b);
  return c;
}

double DirectAnimatedBlockImage::Block::GetX(double t) const {
  return (1 - t) * start_x_ + t * finish_x_;
}

double DirectAnimatedBlockImage::Block::GetY(double t) const {
  return (1 - t) * start_y_ + t * finish_y_;
}

DirectAnimatedBlockImage::DirectAnimatedBlockImage(
    const BlockImage& image1,
    const BlockImage& image2,
    const std::vector<int>& assignment)
    : rows_(image1.rows()),
      cols_(image1.cols()) {
  double rr = 1.0 / rows_;
  double rc = 1.0 / cols_;
  for (int i = 0; i < rows_ * cols_; ++i) {
    int start_r = i / cols_;
    int start_c = i % cols_;
    int end_r = assignment[i] / cols_;
    int end_c = assignment[i] % cols_;
    blocks_.emplace_back(
        image1.blocks()[i],
        image2.blocks()[assignment[i]],
        rc * start_c, rr * (rows_ - start_r - 1),
        rc * end_c, rr * (rows_ - end_r - 1));
  }
}

void DirectAnimatedBlockImage::Draw(int h, double t) {
  glBegin(GL_QUADS);
  for (const Block& block : blocks_) {
    const Color& c = block.GetColor(t);
    glColor3ub(c.r, c.g, c.b);
    int x = round(cols_ * h * block.GetX(t));
    int y = round(rows_ * h * block.GetY(t));
    glVertex2f(x, y);
    glVertex2f(x + h, y);
    glVertex2f(x + h, y + h);
    glVertex2f(x, y + h);
  }
  glEnd();

}

}  // namespace pixeldance
