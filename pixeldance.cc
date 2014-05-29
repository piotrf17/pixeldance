#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <GL/gl.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "animated_block_image.h"
#include "block_image.h"
#include "direct_animated_block_image.h"
#include "window.h"

using pixeldance::AnimatedBlockImage;
using pixeldance::Color;
using pixeldance::DirectAnimatedBlockImage;
using pixeldance::BlockImage;

namespace {

class PixeldanceWindow : public graphics::Window2d {
 public:
  PixeldanceWindow(std::unique_ptr<AnimatedBlockImage> image)
      : graphics::Window2d(800, 600, "pixeldance"),
        image_(std::move(image)), reversible_(image_->reversible()),
        t_(0.0), dt_(reversible_ ? 0.0 : 0.01) {}
  virtual ~PixeldanceWindow() {}

 protected:
  virtual void Keypress(unsigned int key) {
    switch (key) {
      case XK_Escape:
        Close();
        break;
      case XK_space:
        if (reversible_) {
          if (t_ > 0.5) {
            dt_ = -0.01;
          } else {
            dt_ = 0.01;
          }
        }
        break;
    }
  }

  virtual void Draw() {
    glClear(GL_COLOR_BUFFER_BIT);
    glPushMatrix();
    glTranslatef(150, 100, 0);
    image_->Draw(25, t_);
    glPopMatrix();
    t_ += dt_;
    if (reversible_) {
      if (t_ < 0.0 || t_ > 1.0) {
        t_ = std::max(std::min(t_, 1.0), 0.0);
        dt_ = 0.0;
      }
    }
  }

 private:
  std::unique_ptr<AnimatedBlockImage> image_;
  bool reversible_;
  double t_;
  double dt_;
};

int ColorDistance(const Color& x, const Color& y) {
  return
      (x.r - y.r) * (x.r - y.r) +
      (x.g - y.g) * (x.g - y.g) +
      (x.b - y.b) * (x.b - y.b);
}

int Cost(std::vector<std::vector<int>> cost_matrix,
         std::vector<int> a) {
  int cost = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    cost += cost_matrix[i][a[i]];
  }
  return cost;
}

// Stupid random assignment algorithm.
std::vector<int> Assign(std::vector<std::vector<int>> cost_matrix) {
  std::vector<int> a(cost_matrix.size());
  for (size_t i = 0; i < a.size(); ++i) {
    a[i] = i;
  }
  std::shuffle(a.begin(), a.end(), std::default_random_engine(22));
  int cur_cost = Cost(cost_matrix, a);
  printf("initial cost = %d\n", cur_cost);
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(0, a.size() - 1);
  for (int iter = 0; iter < 500000; ++iter) {
    int i = distribution(generator);
    int j = distribution(generator);
    if (i == j) {
      continue;
    }
    std::swap(a[i], a[j]);
    int new_cost = Cost(cost_matrix, a);
    if (new_cost < cur_cost) {
      cur_cost = new_cost;
    } else {
      std::swap(a[i], a[j]);
    }
  }
  printf("final cost= %d\n", cur_cost);
  return a;
}

void ComputeBlockMap(const BlockImage& image1,
                     const BlockImage& image2,
                     std::vector<int>* block_map) {
  // Build a matrix of color distances between blocks in each image.
  // Matrix is symmetric, and yet we compute both sides for fun!
  assert(image1.blocks().size() == image2.blocks().size());
  const int num_blocks = image1.blocks().size();
  std::vector<std::vector<int>> cost_matrix(num_blocks);
  for (int i = 0; i < num_blocks; ++i) {
    cost_matrix[i].resize(num_blocks);
    for (int j = 0; j < num_blocks; ++j) {
      cost_matrix[i][j] = ColorDistance(image1.blocks()[i],
                                        image2.blocks()[j]);
    }
  }
  printf("costs computed!\n");
  *block_map = Assign(cost_matrix);
}

// for quick experimentation:
void SaveBlockMap(const std::vector<int>& block_map,
                  const std::string& filename) {
  std::ofstream outfile(filename);
  outfile << block_map.size() << std::endl;
  for (int i : block_map) {
    outfile << i << std::endl;
  }
}

void LoadBlockMap(const std::string& filename,
                  std::vector<int>* block_map) {
  std::ifstream infile(filename);
  int size;
  infile >> size;
  block_map->resize(size);
  for (int i = 0; i < size; ++i) {
    infile >> (*block_map)[i];
  }
}

}  // namespace

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  BlockImage image1(cv::imread("../image1.jpg", CV_LOAD_IMAGE_COLOR),
                    16, 20);
  BlockImage image2(cv::imread("../image2.jpg", CV_LOAD_IMAGE_COLOR),
                    16, 20);

  std::vector<int> block_map;
//   ComputeBlockMap(image1, image2, &block_map);
//   SaveBlockMap(block_map, "block_map.txt");
//   return 0;
  LoadBlockMap("block_map.txt", &block_map);

  std::unique_ptr<AnimatedBlockImage> animation(
      new DirectAnimatedBlockImage(image1, image2, block_map));
  
  PixeldanceWindow window(std::move(animation));
  window.Run();
  
  return 0;
}
