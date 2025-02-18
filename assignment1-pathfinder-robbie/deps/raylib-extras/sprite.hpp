#ifndef _SPRITE_HPP_
#define _SPRITE_HPP_

// version 1.0.0

#include "raylib-cpp.hpp"
#include <vector>
#include <map> // unordered_map would need a hash function for std::vector<int>
#include <utility>
#include <string>

namespace aipfg {

struct Sprite
{
  Sprite(const raylib::Texture& tex,
         const int ncols = 1, const int nrows = 1, const Vector2 posn = {},
         const std::map<std::string, std::vector<int>>& all_frame_ids = {{ "", { 0 } }},
         const int sprite_fps = 0)
    : tex_{ tex },
      all_frame_ids_{ all_frame_ids },
      ncols_{ ncols },
      nrows_{ nrows },
      posn_{ posn },
      sprite_fps_{ sprite_fps },
      sprite_width_{ tex_.width / ncols_ },
      sprite_height_{ tex_.height / nrows_ },
      prev_time_{ GetTime() }
      {
        state_     = all_frame_ids_.begin()->first;
        frame_ids_ = &(all_frame_ids_.begin()->second);
        frame_rec_ = calc_frame_rect((*frame_ids_)[0]);
      }

  auto get_num_frames() { return (*frame_ids_).size(); }
  void set_state(const std::string& new_state)
  {
    if (all_frame_ids_.count(new_state) > 0) // i.e. an animation is associated
    {
      animation_on_ = true;

      if (new_state != state_)
      {
        frame_ids_ = &all_frame_ids_[new_state];
        frame_rec_ = calc_frame_rect((*frame_ids_)[0]);
      }
    }

    state_ = new_state;
  }

  void update()
  {
    if (animation_on_)
    {
      update_animation();
    }
  }

  void draw()
  {
    const float w = static_cast<float>(sprite_width_)  * scale_;
    const float h = static_cast<float>(sprite_height_) * scale_;
    const Rectangle dest{ posn_.x, posn_.y, w, h };
    Vector2 origin{ w * centre_.x, h * centre_.y };
    DrawTexturePro(tex_, frame_rec_, dest, origin, angle_ += rotation_, WHITE);
  }
  void draw(const Vector2 posn) { posn_ = posn; draw(); }
  void draw_cell(const int x, const int y)
  {
    posn_.x = static_cast<float>(x) * sprite_width_;
    posn_.y = static_cast<float>(y) * sprite_height_;
    draw();
  }
  void draw_cell(const int x, const int y, const int frame_id)
  {
    calc_frame_rect((*frame_ids_)[frame_id]);
    draw_cell(x, y);
  }

private:

  Rectangle calc_frame_rect(const int frame_id)
  {
    const int frame_x = frame_id % ncols_;
    const int frame_y = frame_id / ncols_;
    return { (float)frame_x * sprite_width_, (float)frame_y * sprite_height_,
             (float)sprite_width_, (float)sprite_height_ };
  }

  void update_animation()
  {
    const double t = GetTime();
    const double time_diff = t - prev_time_;
    const double one_frame = 1.0 / sprite_fps_;
    const int nframes = static_cast<int>(time_diff / one_frame);
    const int temp_frame = (frame_ + nframes) % (*frame_ids_).size(); // consider ALT+TAB

    frame_rec_ = calc_frame_rect((*frame_ids_)[temp_frame]);
    const double anim_cycle_dur = (*frame_ids_).size() / (double)sprite_fps_;
    if (time_diff > anim_cycle_dur)
    {
      frame_ = temp_frame;
      prev_time_ = t;
    }
  }

public:

  Vector2 posn_{ 0, 0 };
  float scale_{ 1.0f };
  float angle_{ 0 };
  float rotation_{ 0 }; // the change in angle applied each frame
  Vector2 centre_{ 0.0f, 0.0f };  // normalised [0-1] "centre" of the sprite
  bool animation_on_{ false };

private:

  const raylib::Texture& tex_;
  const int ncols_, nrows_;
  const int sprite_width_, sprite_height_;
  int sprite_fps_;
  double prev_time_;
  int frame_{ 0 }; // zero-based frame index for frame_ids_
  std::string state_; // can possibly be a state not within all_frame_ids_
  std::map<std::string, std::vector<int>> all_frame_ids_;
  std::vector<int>* frame_ids_; // flattened coordinate of each frame
  Rectangle frame_rec_;
};

} // namespace aipfg

#endif // _SPRITE_HPP_
