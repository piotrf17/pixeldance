// An animation that just directly slides the blocks from one image into the
// other along a straight line.  Trivially reversible.
#ifndef PIXELDANCE_DIRECT_ANIMATED_BLOCK_IMAGE_H_
#define PIXELDANCE_DIRECT_ANIMATED_BLOCK_IMAGE_H_

#include <vector>

#include "animated_block_image.h"

namespace pixeldance {

class BlockImage;

class DirectAnimatedBlockImage : public AnimatedBlockImage {
 public:
  virtual bool reversible() const { return true; }

  DirectAnimatedBlockImage(const BlockImage& image1,
                           const BlockImage& image2,
                           const std::vector<int>& assignment);
  
  virtual void Draw(int h, double t);

 private:
  class Block {
   public:
    Block(const Color& start_color,
          const Color& finish_color,
          double start_x, double start_y,
          double finish_x, double finish_y);

    // Given t between 0 and 1, interpolate a block value.
    Color GetColor(double t) const;
    double GetX(double t) const;
    double GetY(double t) const;

   private:
    Color start_color_;
    Color finish_color_;
    double start_x_, start_y_;
    double finish_x_, finish_y_;
  };
  
  int rows_;
  int cols_;
  std::vector<Block> blocks_;
};

}  // namespace pixeldance

#endif // PIXELDANCE_DIRECT_ANIMATED_BLOCK_IMAGE_H_
