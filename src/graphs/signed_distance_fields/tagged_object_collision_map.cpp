#include "tagged_object_collision_map.hpp"
using namespace sdf_tools;

#ifdef ROS_EXISTS
sdf_tools::TaggedObjectCollisionMap
TaggedObjectCollisionMapGrid::GetMessageRepresentation(
    const TaggedObjectCollisionMapGrid& map)
{
  sdf_tools::TaggedObjectCollisionMap map_message;
  map_message.header.stamp = ros::Time::now();
  map_message.header.frame_id = map.GetFrame();
  std::vector<uint8_t> buffer;
  map.SerializeSelf(buffer);
  map_message.serialized_map = ZlibHelpers::CompressBytes(buffer);
  map_message.is_compressed = true;
  return map_message;
}

TaggedObjectCollisionMapGrid
TaggedObjectCollisionMapGrid::LoadFromMessageRepresentation(
    const sdf_tools::TaggedObjectCollisionMap& message)
{
  if (message.is_compressed)
  {
    const std::vector<uint8_t> uncompressed_map
        = ZlibHelpers::DecompressBytes(message.serialized_map);
    TaggedObjectCollisionMapGrid map;
    map.DeserializeSelf(uncompressed_map, 0);
    return map;
  }
  else
  {
    TaggedObjectCollisionMapGrid map;
    map.DeserializeSelf(message.serialized_map, 0);
    return map;
  }
}
#endif

uint32_t TaggedObjectCollisionMapGrid::UpdateConnectedComponents()
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
      SetValue(index, TAGGED_OBJECT_COLLISION_CELL(query.first.occupancy,
                                                   query.first.object_id,
                                                   component,
                                                   query.first.convex_segment));
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

TaggedObjectCollisionMapGrid
TaggedObjectCollisionMapGrid::Resample(const double new_resolution) const
{
  TaggedObjectCollisionMapGrid resampled(GetOriginTransform(),
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
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        const Eigen::Vector4d current_cell_location
            = GridIndexToLocation(x_index, y_index, z_index);
        resampled.SetValue4d(current_cell_location, current_cell);
      }
    }
  }
  return resampled;
}

std::map<uint32_t, std::pair<int32_t, int32_t>>
TaggedObjectCollisionMapGrid::ComputeComponentTopology(
    const COMPONENT_TYPES component_types_to_use,
    const bool recompute_connected_components,
    const bool verbose)
{
  // Recompute the connected components if need be
  if (recompute_connected_components)
  {
    UpdateConnectedComponents();
  }
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
    const TAGGED_OBJECT_COLLISION_CELL& current_cell
        = GetImmutable(index).first;
    if (current_cell.occupancy > 0.5)
    {
      if ((component_types_to_use & FILLED_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.y))
        {
          return true;
        }
      }
    }
    else if (current_cell.occupancy < 0.5)
    {
      if ((component_types_to_use & EMPTY_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.y, index.z))
        {
          return true;
        }
      }
    }
    else
    {
      if ((component_types_to_use & UNKNOWN_COMPONENTS) > 0x00)
      {
        if (IsConnectedComponentSurfaceIndex(index.x, index.z, index.z))
        {
          return true;
        }
      }
    }
    return false;
  };
  return topology_computation::ComputeComponentTopology(*this,
                                                        get_component_fn,
                                                        is_surface_index_fn,
                                                        verbose);
}

std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>>
TaggedObjectCollisionMapGrid::ExtractComponentSurfaces(
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
    const TAGGED_OBJECT_COLLISION_CELL& current_cell
        = GetImmutable(index).first;
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

uint32_t TaggedObjectCollisionMapGrid::UpdateConvexSegments(
    const double connected_threshold,
    const bool add_virtual_border)
{
  const auto sdf_result
      = (add_virtual_border) ?
          ExtractSignedDistanceField(
            std::numeric_limits<float>::infinity(),
            std::vector<uint32_t>(),
            true, true) :
          ExtractFreeAndNamedObjectsSignedDistanceField(
            std::numeric_limits<float>::infinity(), true);
  const SignedDistanceField& sdf = sdf_result.first;
  const VoxelGrid<Eigen::Vector3d> extrema_map = sdf.ComputeLocalExtremaMap();
  // Make the helper functions
  // This is not enough, we also need to limit the curvature of the
  // segment/local extrema cluster! Otherwise thin objects will always have
  // their local extrema dominated by their surroundings rather than their
  // own structure!
  const std::function<bool(const GRID_INDEX&, const GRID_INDEX&)>
    are_connected_fn
      = [&] (const GRID_INDEX& index1, const GRID_INDEX& index2)
  {
    auto query1 = GetImmutable(index1);
    auto query2 = GetImmutable(index2);
    assert(query1.second);
    assert(query2.second);
    if (query1.first.object_id == query2.first.object_id)
    {
      auto exmap_query1 = extrema_map.GetImmutable(index1);
      auto examp_query2 = extrema_map.GetImmutable(index2);
      assert(exmap_query1.second);
      assert(examp_query2.second);
      const double maxima_distance
          = (exmap_query1.first - examp_query2.first).norm();
      if (maxima_distance < connected_threshold)
      {
        return true;
      }
      else
      {
        return false;
      }
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
    auto extrema_query = extrema_map.GetImmutable(index);
    if (query.second)
    {
      if ((query.first.occupancy < 0.5f) || (query.first.object_id > 0u))
      {
        const Eigen::Vector3d& extrema = extrema_query.first;
        if (!std::isinf(extrema.x())
            && !std::isinf(extrema.y())
            && !std::isinf(extrema.z()))
        {
          return (int64_t)query.first.convex_segment;
        }
        else
        {
          // Ignore cells with infinite extrema
          return (int64_t)-1;
        }
      }
      else
      {
        // Ignore filled cells that don't belong to an object
        return (int64_t)-1;
      }
    }
    else
    {
      return (int64_t)-1;
    }
  };
  const std::function<void(const GRID_INDEX&, const uint32_t)>
    mark_component_fn
      = [&] (const GRID_INDEX& index, const uint32_t component)
  {
    auto query = GetMutable(index);
    if (query.second)
    {
      SetValue(index, TAGGED_OBJECT_COLLISION_CELL(query.first.occupancy,
                                                   query.first.object_id,
                                                   query.first.component,
                                                   component));
    }
  };
  number_of_convex_segments_
      = topology_computation::ComputeConnectedComponents(*this,
                                                         are_connected_fn,
                                                         get_component_fn,
                                                         mark_component_fn);
  convex_segments_valid_ = true;
  return number_of_convex_segments_;
}

#ifdef ROS_EXISTS
visualization_msgs::Marker TaggedObjectCollisionMapGrid::ExportForDisplay(
    const float alpha, const std::vector<uint32_t>& objects_to_draw) const
{
  std::map<uint32_t, uint32_t> objects_to_draw_map;
  for (size_t idx = 0; idx < objects_to_draw.size(); idx++)
  {
    objects_to_draw_map[objects_to_draw[idx]] = 1u;
  }
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_collision_map_display";
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
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        const auto draw_found_itr
            = objects_to_draw_map.find(current_cell.object_id);
        if (draw_found_itr != objects_to_draw_map.end()
            || objects_to_draw_map.size() == 0)
        {
          const std_msgs::ColorRGBA object_color
              = GenerateComponentColor(current_cell.object_id, alpha);
          if (object_color.a > 0.0)
          {
            display_rep.points.push_back(new_point);
            display_rep.colors.push_back(object_color);
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::Marker TaggedObjectCollisionMapGrid::ExportForDisplay(
    const std::map<uint32_t, std_msgs::ColorRGBA>& object_color_map) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_collision_map_display";
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
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        // Check if we've been given a color to work with
        auto found_itr = object_color_map.find(current_cell.object_id);
        std_msgs::ColorRGBA object_color;
        if (found_itr != object_color_map.end())
        {
          object_color = found_itr->second;
        }
        else
        {
          object_color = GenerateComponentColor(current_cell.object_id);
        }
        if (object_color.a > 0.0)
        {
          display_rep.points.push_back(new_point);
          display_rep.colors.push_back(object_color);
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportContourOnlyForDisplay(
    const float alpha, const std::vector<uint32_t>& objects_to_draw) const
{
  std::map<uint32_t, uint32_t> objects_to_draw_map;
  for (size_t idx = 0; idx < objects_to_draw.size(); idx++)
  {
      objects_to_draw_map[objects_to_draw[idx]] = 1u;
  }
  // Make SDF
  const std::map<uint32_t, sdf_tools::SignedDistanceField> per_object_sdfs
      = (objects_to_draw.size() > 0) ?
          MakeObjectSDFs(objects_to_draw, true, true)
        : MakeAllObjectSDFs(true, false);
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_collision_map_display";
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
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        // Get the SDF for the current object
        auto sdf_found_itr = per_object_sdfs.find(current_cell.object_id);
        if (sdf_found_itr != per_object_sdfs.end())
        {
          const sdf_tools::SignedDistanceField& object_sdf
              = sdf_found_itr->second;
          const float distance
              = object_sdf.GetImmutable(new_point.x,
                                        new_point.y,
                                        new_point.z).first;
          // Check if we're on the surface of the object
          if (distance < 0.0 && distance > -GetResolution())
          {
            const auto draw_found_itr
                = objects_to_draw_map.find(current_cell.object_id);
            if (draw_found_itr != objects_to_draw_map.end()
                || objects_to_draw_map.size() == 0)
            {
              const std_msgs::ColorRGBA object_color
                  = GenerateComponentColor(current_cell.object_id, alpha);
              if (object_color.a > 0.0)
              {
                display_rep.points.push_back(new_point);
                display_rep.colors.push_back(object_color);
              }
            }
          }
        }
      }
    }
  }
  return display_rep;
}

// Note that this function will use a default color for objects that a color
// is not provided for (other than object ID 0, which is ignored by
// MakeAllObjectSDFs)
visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportContourOnlyForDisplay(
    const std::map<uint32_t, std_msgs::ColorRGBA>& object_color_map) const
{
  // Make SDFs for the objects that we will be displaying
  const std::map<uint32_t, sdf_tools::SignedDistanceField> per_object_sdfs
      = MakeAllObjectSDFs(true, false);
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_collision_map_display";
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
  // 1.9 is to ensure that we get all corners, without also getting extra
  // interior cells
  const float bdy_cell_min_dist = -1.9 * GetResolution();
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
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        // Get the SDF for the current object
        auto sdf_found_itr = per_object_sdfs.find(current_cell.object_id);
        if (sdf_found_itr != per_object_sdfs.end())
        {
          const sdf_tools::SignedDistanceField& object_sdf
              = sdf_found_itr->second;
          const float distance
              = object_sdf.GetImmutable(x_index, y_index, z_index).first;
          // Check if we're on the surface of the object
          if (distance < 0.0 && distance > bdy_cell_min_dist)
          {
            // Check if we've been given a color to work with
            auto found_itr = object_color_map.find(current_cell.object_id);
            std_msgs::ColorRGBA object_color;
            if (found_itr != object_color_map.end())
            {
              object_color = found_itr->second;
            }
            else
            {
              object_color = GenerateComponentColor(current_cell.object_id);
            }
            if (object_color.a > 0.0)
            {
              display_rep.points.push_back(new_point);
              display_rep.colors.push_back(object_color);
            }
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportForDisplayOccupancyOnly(
    const std_msgs::ColorRGBA& collision_color,
    const std_msgs::ColorRGBA& free_color,
    const std_msgs::ColorRGBA& unknown_color) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_collision_map_occupancy_display";
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

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportConnectedComponentsForDisplay(
    const bool color_unknown_components) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_connected_components_display";
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
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
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

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportConvexSegmentForDisplay(
    const uint32_t object_id, const uint32_t convex_segment) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_"
                   + std::to_string(object_id)
                   + "_convex_segment_"
                   + std::to_string(convex_segment)
                   + "_display";
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
        const TAGGED_OBJECT_COLLISION_CELL& current_cell
            = GetImmutable(x_index, y_index, z_index).first;
        if ((current_cell.object_id == object_id)
            && (current_cell.convex_segment == convex_segment))
        {
          // Convert grid indices into a real-world location
          const Eigen::Vector4d location
              = GridIndexToLocationGridFrame(x_index, y_index, z_index);
          geometry_msgs::Point new_point;
          new_point.x = location(0);
          new_point.y = location(1);
          new_point.z = location(2);
          display_rep.points.push_back(new_point);
          // Generate a color
          if (number_of_convex_segments_ < 22)
          {
            const std_msgs::ColorRGBA color
                = GenerateComponentColor(convex_segment);
            display_rep.colors.push_back(color);
          }
          else
          {
            const std_msgs::ColorRGBA color
                = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>
                  ::InterpolateHotToCold(convex_segment, 1.0,
                                         (double)number_of_convex_segments_);
            display_rep.colors.push_back(color);
          }
        }
      }
    }
  }
  return display_rep;
}

visualization_msgs::Marker
TaggedObjectCollisionMapGrid::ExportSurfaceForDisplay(
    const std::unordered_map<GRID_INDEX, uint8_t>& surface,
    const std_msgs::ColorRGBA& surface_color) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "tagged_object_collision_map_surface";
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
  // Add all the cells of the surface
  std::unordered_map<GRID_INDEX, uint8_t>::const_iterator surface_itr;
  for (surface_itr = surface.begin();
       surface_itr != surface.end();
       ++surface_itr)
  {
    const GRID_INDEX index = surface_itr->first;
    const int8_t validity = surface_itr->second;
    if (validity == 1)
    {
      // Convert grid indices into a real-world location
      const Eigen::Vector4d location = GridIndexToLocationGridFrame(index);
      geometry_msgs::Point new_point;
      new_point.x = location(0);
      new_point.y = location(1);
      new_point.z = location(2);
      display_rep.points.push_back(new_point);
      display_rep.colors.push_back(surface_color);
    }
  }
  return display_rep;
}
#endif
