#ifndef TA11_DISPLAY_H
#define TA11_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#endif

#include "tiago_tactile_msgs/TA11.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace ta11_viz
{

class TA11Visual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// TA11Display will show a 3D arrow showing the direction and magnitude
// of the TA11 force sensor.

class TA11Display: public rviz::MessageFilterDisplay<tiago_tactile_msgs::TA11>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  TA11Display();
  virtual ~TA11Display();

protected:
  virtual void onInitialize();
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateArrowScale();
  void updateColorAndAlpha();
  void updateForceThreshold();

  // Function to handle an incoming ROS message.
private:
  void processMessage( const tiago_tactile_msgs::TA11::ConstPtr& msg );
  Ogre::SceneNode* right_frame_;
  Ogre::SceneNode* left_frame_;

  std::shared_ptr<rviz::Arrow> right_arrow_;
  std::shared_ptr<rviz::Arrow> left_arrow_;

  float arrow_scale_, force_threshold_;

  // User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* arrow_scale_property_;
  rviz::FloatProperty* force_threshold_property_;
};

}

#endif
