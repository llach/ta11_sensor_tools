#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "ta11_display.h"

namespace ta11_viz
{

TA11Display::TA11Display()
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the force arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  force_threshold_property_ = new rviz::FloatProperty( "Force Threshold", 0.1,
                                                   "Forces below this threshold won't be visualized.",
                                                   this, SLOT( updateForceThreshold() ));

  arrow_scale_property_ = new rviz::FloatProperty( "Arrow Scale", 2,
                                             "Additional force arrow scaling factor.",
                                             this, SLOT( updateArrowScale() ));

  arrow_scale_ = 0.0;
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void TA11Display::onInitialize()
{
  MFDClass::onInitialize();
  right_frame_ =  scene_node_->createChildSceneNode();
  left_frame_ =  scene_node_->createChildSceneNode();

  right_arrow_.reset(new rviz::Arrow( scene_manager_, right_frame_ ));
  left_arrow_.reset(new rviz::Arrow( scene_manager_, left_frame_ ));

  right_arrow_->setDirection({-1, 0, 0});
  left_arrow_->setDirection({1, 0, 0});
  left_arrow_->setScale({0, 0, 0});
  right_arrow_->setScale({0, 0, 0});

  updateColorAndAlpha();
  updateForceThreshold();
}

TA11Display::~TA11Display()
{
}

// Clear the visuals by deleting their objects.
void TA11Display::reset()
{
  MFDClass::reset();
}

// Set the current color and alpha values for each visual.
void TA11Display::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  right_arrow_->setColor( color.r, color.g, color.b, alpha );
  left_arrow_->setColor( color.r, color.g, color.b, alpha );
}

void TA11Display::updateArrowScale()
{
  arrow_scale_ = arrow_scale_property_->getFloat();
}

void TA11Display::updateForceThreshold()
{
  force_threshold_ = force_threshold_property_->getFloat();
}

// This is our callback to handle an incoming message.
void TA11Display::processMessage( const tiago_tactile_msgs::TA11::ConstPtr& msg )
{
  if (msg->sensor_values.size() != msg->frame_names.size()){
    ROS_ERROR("msg->sensor_values.size() != msg->frame_names.size() with %lu and %lu", msg->sensor_values.size(), msg->frame_names.size());
  }
  for (int i=0;i<msg->sensor_values.size();i++) {

    std::shared_ptr<rviz::Arrow> ar = nullptr;
    Ogre::SceneNode* sn = nullptr;
    std::string sensor_frame = msg->frame_names[i];

    if (sensor_frame == "ta11_left_finger_link") {
        ar = left_arrow_;
        sn = left_frame_;
    } else if (sensor_frame == "ta11_right_finger_link") {
        ar = right_arrow_;
        sn = right_frame_;
    } else {
        ROS_ERROR_STREAM("Unknown sensor frame " << sensor_frame);
    }

    float length = msg->sensor_values[i];
    if (abs(msg->sensor_values[i]) < force_threshold_){
      length = 0;
    }
    Ogre::Vector3 scale( length, length, length );
    ar->setScale( arrow_scale_*scale );

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(sensor_frame,
                                                   msg->header.stamp,
                                                   position, orientation)) {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                  sensor_frame.c_str(), qPrintable(fixed_frame_));
        continue;
    }

    sn->setOrientation(orientation);
    sn->setPosition(position);
   updateColorAndAlpha();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ta11_viz::TA11Display,rviz::Display )
