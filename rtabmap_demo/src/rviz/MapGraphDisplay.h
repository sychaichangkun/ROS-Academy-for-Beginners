/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef MAP_GRAPH_DISPLAY_H
#define MAP_GRAPH_DISPLAY_H

#include <rtabmap_ros/MapGraph.h>

#include <rviz/message_filter_display.h>

namespace Ogre
{
class ManualObject;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
}

using namespace rviz;

namespace rtabmap_ros
{

/**
 * \class MapGraphDisplay
 * \brief Displays the graph of rtabmap::MapGraph message
 */
class MapGraphDisplay: public MessageFilterDisplay<rtabmap_ros::MapGraph>
{
Q_OBJECT
public:
  MapGraphDisplay();
  virtual ~MapGraphDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage( const rtabmap_ros::MapGraph::ConstPtr& msg );

private:
  void destroyObjects();

  std::vector<Ogre::ManualObject*> manual_objects_;

  ColorProperty* color_neighbor_property_;
  ColorProperty* color_neighbor_merged_property_;
  ColorProperty* color_global_property_;
  ColorProperty* color_local_property_;
  ColorProperty* color_user_property_;
  ColorProperty* color_virtual_property_;
  FloatProperty* alpha_property_;
};

} // namespace rtabmap_ros

#endif /* MAP_GRAPH_DISPLAY_H */

