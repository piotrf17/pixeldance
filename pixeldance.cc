#include <cstdint>
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

#include "window.h"

namespace {

struct Color {
  Color() : r(0), g(0), b(0), a(255) {}
  uint8_t r, g, b, a;
};

class BlockImage {
 public:
  BlockImage(const cv::Mat& original, int rows, int cols)
      : original_(original),
        rows_(rows),
        cols_(cols) {
    Build();
  }

  void Draw(int h) const {
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

  std::vector<Color> blocks() const {
    return blocks_;
  }

  int rows() const { return rows_; }
  int cols() const { return cols_; }

 private:
  void Build() {
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
  
  cv::Mat original_;
  int rows_, cols_;
  std::vector<Color> blocks_;
};

class Block {
 public:
  Block(const Color& start_color,
        const Color& finish_color,
        double start_x, double start_y,
        double finish_x, double finish_y)
      : start_color_(start_color),
        finish_color_(finish_color),
        start_x_(start_x), start_y_(start_y),
        finish_x_(finish_x), finish_y_(finish_y) {
  }

  Color GetColor(double t) const {
    Color c;
    c.r = round((1 - t) * start_color_.r + t * finish_color_.r);
    c.g = round((1 - t) * start_color_.g + t * finish_color_.g);
    c.b = round((1 - t) * start_color_.b + t * finish_color_.b);
    return c;
  }

  double GetX(double t) const {
    return (1 - t) * start_x_ + t * finish_x_;
  }

  double GetY(double t) const {
    return (1 - t) * start_y_ + t * finish_y_;
  }

 private:
  Color start_color_;
  Color finish_color_;
  double start_x_, start_y_;
  double finish_x_, finish_y_;
};

class AnimatedBlockImage {
 public:
  AnimatedBlockImage(const BlockImage& image1,
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

  void Draw(int h, double t) const {
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

 private:
  int rows_;
  int cols_;
  std::vector<Block> blocks_;
};

class PixeldanceWindow : public graphics::Window2d {
 public:
  PixeldanceWindow(const AnimatedBlockImage* image)
      : graphics::Window2d(800, 600, "pixeldance"),
        image_(image), t_(0.0), dt_(0.0) {}
  virtual ~PixeldanceWindow() {}

 protected:
  virtual void Keypress(unsigned int key) {
    switch (key) {
      case XK_Escape:
        Close();
        break;
      case XK_space:
        if (t_ > 0.5) {
          dt_ = -0.01;
        } else {
          dt_ = 0.01;
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
    if (t_ < 0.0 || t_ > 1.0) {
      t_ = std::max(std::min(t_, 1.0), 0.0);
      dt_ = 0.0;
    }
  }

 private:
  const AnimatedBlockImage* image_;
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

  AnimatedBlockImage animation(image1, image2, block_map);
  
  PixeldanceWindow window(&animation);
  window.Run();
  
  return 0;
}
