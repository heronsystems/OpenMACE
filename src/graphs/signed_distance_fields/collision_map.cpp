#include "collision_map.hpp"

using namespace sdf_tools;

#ifdef ROS_EXISTS
sdf_tools::CollisionMap CollisionMapGrid::GetMessageRepresentation(
    const CollisionMapGrid& map)
{
  sdf_tools::CollisionMap map_message;
  map_message.header.stamp = ros::Time::now();
  map_message.header.frame_id = map.GetFrame();
  std::vector<uint8_t> buffer;
  map.SerializeSelf(buffer);
  map_message.serialized_map = ZlibHelpers::CompressBytes(buffer);
  map_message.is_compressed = true;
  return map_message;
}

CollisionMapGrid CollisionMapGrid::LoadFromMessageRepresentation(
    const sdf_tools::CollisionMap& message)
{
  if (message.is_compressed)
  {
    const std::vector<uint8_t> uncompressed_map
        = ZlibHelpers::DecompressBytes(message.serialized_map);
    CollisionMapGrid map;
    map.DeserializeSelf(uncompressed_map, 0);
    return map;
  }
  else
  {
    CollisionMapGrid map;
    map.DeserializeSelf(message.serialized_map, 0);
    return map;
  }
}

visualization_msgs::Marker CollisionMapGrid::ExportForDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "collision_map_display";
  display_rep.id = 1;
  display_rep.type = visualization_msgs::Marker::CUBE_LIST;
  display_rep.action = visualization_msgs::Marker::ADD;
  display_rep.lifetime = ros::Duration(0.0);
  display_rep.frame_locked = false;
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Convert grid indices into a real-world location
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(x_index, y_index, z_index);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        if (GetImmutable(x_index, y_index, z_index).first.occupancy > 0.5)
        {
          if (collision_color.a > 0.0)
          {
            display_rep.points.push_back(new_point);
            display_rep.colors.push_back(collision_color);
          }
        }
        else if (GetImmutable(x_index, y_index, z_index).first.occupancy < 0.5)
        {
          if (free_color.a > 0.0)
          {
            display_rep.points.push_back(new_point);
            display_rep.colors.push_back(free_color);
          }
        }
        else
        {
          if (unknown_color.a > 0.0)
          {
            display_rep.points.push_back(new_point);
            display_rep.colors.push_back(unknown_color);
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::MarkerArray CollisionMapGrid::ExportForSeparateDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  const std_msgs::ColorRGBA no_color
      = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>
        ::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
  visualization_msgs::Marker collision_only_marker
      = ExportForDisplay(collision_color, no_color, no_color);
  collision_only_marker.ns = "collision_only";
  visualization_msgs::Marker free_only_marker
      = ExportForDisplay(no_color, free_color, no_color);
  free_only_marker.ns = "free_only";
  visualization_msgs::Marker unknown_only_marker
      = ExportForDisplay(no_color, no_color, unknown_color);
  unknown_only_marker.ns = "unknown_only";
  visualization_msgs::MarkerArray display_messages;
  display_messages.markers = {collision_only_marker,
                              free_only_marker,
                              unknown_only_marker};
  return display_messages;
}

visualization_msgs::Marker CollisionMapGrid::ExportSurfacesForDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "collision_map_display";
  display_rep.id = 1;
  display_rep.type = visualization_msgs::Marker::CUBE_LIST;
  display_rep.action = visualization_msgs::Marker::ADD;
  display_rep.lifetime = ros::Duration(0.0);
  display_rep.frame_locked = false;
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        if (IsSurfaceIndex(x_index, y_index, z_index))
        {
          // Convert grid indices into a real-world location
          const Eigen::Vector4d location
              = GridIndexToLocationGridFrame(x_index, y_index, z_index);
          geometry_msgs::Point new_point;
          new_point.x = location(0);
          new_point.y = location(1);
          new_point.z = location(2);
          if (GetImmutable(x_index, y_index, z_index).first.occupancy > 0.5)
          {
            if (collision_color.a > 0.0)
            {
              display_rep.points.push_back(new_point);
              display_rep.colors.push_back(collision_color);
            }
          }
          else if (GetImmutable(x_index, y_index, z_index).first.occupancy
                   < 0.5)
          {
            if (free_color.a > 0.0)
            {
              display_rep.points.push_back(new_point);
              display_rep.colors.push_back(free_color);
            }
          }
          else
          {
            if (unknown_color.a > 0.0)
            {
              display_rep.points.push_back(new_point);
              display_rep.colors.push_back(unknown_color);
            }
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::MarkerArray
CollisionMapGrid::ExportSurfacesForSeparateDisplay(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  const std_msgs::ColorRGBA no_color
      = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>
        ::MakeFromFloatColors(0.0, 0.0, 0.0, 0.0);
  visualization_msgs::Marker collision_only_marker
      = ExportSurfacesForDisplay(collision_color, no_color, no_color);
  collision_only_marker.ns = "collision_surfaces_only";
  visualization_msgs::Marker free_only_marker
      = ExportSurfacesForDisplay(no_color, free_color, no_color);
  free_only_marker.ns = "free_surfaces_only";
  visualization_msgs::Marker unknown_only_marker
      = ExportSurfacesForDisplay(no_color, no_color, unknown_color);
  unknown_only_marker.ns = "unknown_surfaces_only";
  visualization_msgs::MarkerArray display_messages;
  display_messages.markers = {collision_only_marker,
                              free_only_marker,
                              unknown_only_marker};
  return display_messages;
}

visualization_msgs::Marker
CollisionMapGrid::ExportConnectedComponentsForDisplay(
    const bool color_unknown_components) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "connected_components_display";
  display_rep.id = 1;
  display_rep.type = visualization_msgs::Marker::CUBE_LIST;
  display_rep.action = visualization_msgs::Marker::ADD;
  display_rep.lifetime = ros::Duration(0.0);
  display_rep.frame_locked = false;
  display_rep.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                       GetOriginTransform());
  display_rep.scale.x = GetResolution();
  display_rep.scale.y = GetResolution();
  display_rep.scale.z = GetResolution();
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Convert grid indices into a real-world location
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(x_index, y_index, z_index);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        display_rep.points.push_back(new_point);
        const COLLISION_CELL current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        if (current_cell.occupancy != 0.5)
        {
          std_msgs::ColorRGBA color
              = GenerateComponentColor(current_cell.component);
          display_rep.colors.push_back(color);
        }
        else
        {
          if (color_unknown_components)
          {
            std_msgs::ColorRGBA color
                = GenerateComponentColor(current_cell.component);
            display_rep.colors.push_back(color);
          }
          else
          {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
            display_rep.colors.push_back(color);
          }
        }
      }
    }
  }
  return display_rep;
}
#endif

uint32_t CollisionMapGrid::UpdateConnectedComponents()
{
  // If the connected components are already valid, skip computing them again
  if (components_valid_)
  {
    return number_of_components_;
  }
  components_valid_ = false;
  // Make the helper functions
  const std::function<bool(const GRID_INDEX&, const GRID_INDEX&)>
    are_connected_fn = [&] (const GRID_INDEX& index1, const GRID_INDEX& index2)
  {
    auto query1 = GetImmutable(index1);
    auto query2 = GetImmutable(index2);
    assert(query1.second);
    assert(query2.second);
    if ((query1.first.occupancy > 0.5) == (query2.first.occupancy > 0.5))
    {
      return true;
    }
    else
    {
      return false;
    }
  };
  const std::function<int64_t(const GRID_INDEX&)> get_component_fn
      = [&] (const GRID_INDEX& index)
  {
    auto query = GetImmutable(index);
    if (query.second)
    {
      return (int64_t)query.first.component;
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<void(const GRID_INDEX&, const uint32_t)> mark_component_fn
      = [&] (const GRID_INDEX& index, const uint32_t component)
  {
    auto query = GetMutable(index);
    if (query.second)
    {
      SetValue(index, COLLISION_CELL(query.first.occupancy, component));
    }
  };
  number_of_components_
      = topology_computation::ComputeConnectedComponents(*this,
                                                         are_connected_fn,
                                                         get_component_fn,
                                                         mark_component_fn);
  components_valid_ = true;
  return number_of_components_;
}

std::map<uint32_t, std::pair<int32_t, int32_t>>
CollisionMapGrid::ComputeComponentTopology(
    const bool ignore_empty_components,
    const bool recompute_connected_components,
    const bool verbose)
{
  // Recompute the connected components if need be
  if (recompute_connected_components)
  {
    UpdateConnectedComponents();
  }
  const std::function<int64_t(const GRID_INDEX&)> get_component_fn
      = [&] (const GRID_INDEX& index)
  {
    auto query = GetImmutable(index);
    if (query.second)
    {
      return (int64_t)query.first.component;
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<bool(const GRID_INDEX&)> is_surface_index_fn
      = [&] (const GRID_INDEX& index)
  {
    if (ignore_empty_components)
    {
      const COLLISION_CELL& current_cell = GetImmutable(index).first;
      if (current_cell.occupancy > 0.5)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
        {
          return true;
        }
      }
    }
    else
    {
      if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
      {
        return true;
      }
    }
    return false;
  };
  return topology_computation::ComputeComponentTopology(*this,
                                                        get_component_fn,
                                                        is_surface_index_fn,
                                                        verbose);
}

CollisionMapGrid CollisionMapGrid::Resample(const double new_resolution) const
{
  CollisionMapGrid resampled(GetOriginTransform(),
                             GetFrame(),
                             new_resolution,
                             GetXSize(), GetYSize(), GetZSize(),
                             GetOOBValue());
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        const COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        const Eigen::Vector4d current_cell_location
            = GridIndexToLocation(x_index, y_index, z_index);
        resampled.SetValue4d(current_cell_location, current_cell);
      }
    }
  }
  return resampled;
}

std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
CollisionMapGrid::ExtractComponentSurfaces(
    const COMPONENT_TYPES component_types_to_extract) const
{
  // Make the helper functions
  const std::function<int64_t(const GRID_INDEX&)> get_component_fn
      = [&] (const GRID_INDEX& index)
  {
    auto query = GetImmutable(index);
    if (query.second)
    {
      return (int64_t)query.first.component;
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<bool(const GRID_INDEX&)> is_surface_index_fn
      = [&] (const GRID_INDEX& index)
  {
    const COLLISION_CELL& current_cell = GetImmutable(index).first;
    if (current_cell.occupancy > 0.5)
    {
      if ((component_types_to_extract & FILLED_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.y))
        {
          return true;
        }
      }
    }
    else if (current_cell.occupancy < 0.5)
    {
      if ((component_types_to_extract & EMPTY_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
        {
          return true;
        }
      }
    }
    else
    {
      if ((component_types_to_extract & UNKNOWN_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.z, index.z))
        {
          return true;
        }
      }
    }
    return false;
  };
  return topology_computation::ExtractComponentSurfaces(*this,
                                                        get_component_fn,
                                                        is_surface_index_fn);
}
