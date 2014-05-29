// Abstract interface for an animation between 2 block images.
#ifndef PIXELDANCE_ANIMATED_BLOCK_IMAGE_H_
#define PIXELDANCE_ANIMATED_BLOCK_IMAGE_H_

namespace pixeldance {

class AnimatedBlockImage {
 public:
  // If reversible, then the time parameter passed into Draw above
  // will vary between 0 and 1, where 0 means entirely the first image
  // and 1 means entirely the second image.
  virtual bool reversible() const { return false; }
  
  // h is a scale factor, t is a time parameter.
  virtual void Draw(int h, double t) = 0;
};

}  // namespace pixeldance

#endif  // PIXELDANCE_ANIMATED_BLOCK_IMAGE_H_
