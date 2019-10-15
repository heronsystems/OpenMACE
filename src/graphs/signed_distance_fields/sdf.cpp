#include "sdf.hpp"

using namespace sdf_tools;

/////////////////////////////////////////////////////////////////////
// Local extrema map computation
/////////////////////////////////////////////////////////////////////

void SignedDistanceField::FollowGradientsToLocalExtremaUnsafe(
    VoxelGrid<Eigen::Vector3d>& watershed_map,
    const int64_t x_index, const int64_t y_index, const int64_t z_index) const
{
  // First, check if we've already found the local extrema for the current cell
  const Eigen::Vector3d& stored
      = watershed_map.GetImmutable(x_index, y_index, z_index).first;
  if (stored.x() != -std::numeric_limits<double>::infinity()
      && stored.y() != -std::numeric_limits<double>::infinity()
      && stored.z() != -std::numeric_limits<double>::infinity())
  {
    // We've already found it for this cell, so we can skip it
    return;
  }
  // Find the local extrema
  std::vector<double> raw_gradient
      = GetGradient(x_index, y_index, z_index, true);
  Eigen::Vector3d gradient_vector(raw_gradient[0],
                                  raw_gradient[1],
                                  raw_gradient[2]);
  if (GradientIsEffectiveFlat(gradient_vector))
  {
    const Eigen::Vector4d location
        = GridIndexToLocationGridFrame(x_index, y_index, z_index);
    const Eigen::Vector3d local_extrema(location(0), location(1), location(2));
    watershed_map.SetValue(x_index, y_index, z_index, local_extrema);
  }
  else
  {
    // Follow the gradient, one cell at a time, until we reach a local maxima
    std::unordered_map<GRID_INDEX, int8_t> path;
    GRID_INDEX current_index(x_index, y_index, z_index);
    path[current_index] = 1;
    Eigen::Vector3d local_extrema(-std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity());
    while (true)
    {
      if (path.size() == 10000)
      {
        std::cerr << "Warning, gradient path is long (i.e >= 10000 steps)"
                  << std::endl;
      }
      current_index = GetNextFromGradient(current_index, gradient_vector);
      if (path[current_index] != 0)
      {
        // If we've already been here, then we are done
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(current_index);
        local_extrema = Eigen::Vector3d(location(0), location(1), location(2));
        break;
      }
      // Check if we've been pushed past the edge
      if (current_index.x < 0 || current_index.y < 0 || current_index.z < 0
          || current_index.x >= watershed_map.GetNumXCells()
          || current_index.y >= watershed_map.GetNumYCells()
          || current_index.z >= watershed_map.GetNumZCells())
      {
        // We have the "off the grid" local maxima
        local_extrema
            = Eigen::Vector3d(std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity());
        break;
      }
      path[current_index] = 1;
      // Check if the new index has already been checked
      const Eigen::Vector3d& new_stored
          = watershed_map.GetImmutable(current_index).first;
      if (new_stored.x() != -std::numeric_limits<double>::infinity()
          && new_stored.y() != -std::numeric_limits<double>::infinity()
          && new_stored.z() != -std::numeric_limits<double>::infinity())
      {
        // We have the local maxima
        local_extrema = new_stored;
        break;
      }
      else
      {
        raw_gradient = GetGradient(current_index, true);
        gradient_vector = Eigen::Vector3d(raw_gradient[0],
                                          raw_gradient[1],
                                          raw_gradient[2]);
        if (GradientIsEffectiveFlat(gradient_vector))
        {
          // We have the local maxima
          const Eigen::Vector4d location
              = GridIndexToLocationGridFrame(current_index);
          local_extrema
              = Eigen::Vector3d(location(0), location(1), location(2));
          break;
        }
      }
    }
    // Now, go back and mark the entire explored path with the local maxima
    for (auto path_itr = path.begin(); path_itr != path.end(); ++path_itr)
    {
      const GRID_INDEX& index = path_itr->first;
      watershed_map.SetValue(index, local_extrema);
    }
  }
}

bool SignedDistanceField::GradientIsEffectiveFlat(
    const Eigen::Vector3d& gradient) const
{
  // A gradient is at a local maxima if the absolute value of all components
  // (x,y,z) are less than 1/2 SDF resolution
  const double step_resolution = GetResolution() * 0.06125;
  if (std::abs(gradient.x()) <= step_resolution
      && std::abs(gradient.y()) <= step_resolution
      && std::abs(gradient.z()) <= step_resolution)
  {
      return true;
  }
  else
  {
      return false;
  }
}

GRID_INDEX SignedDistanceField::GetNextFromGradient(
    const GRID_INDEX& index,
    const Eigen::Vector3d& gradient) const
{
  // Check if it's inside an obstacle
  const float stored_distance = GetImmutable(index).first;
  Eigen::Vector3d working_gradient = gradient;
  if (stored_distance < 0.0)
  {
    working_gradient = gradient * -1.0;
  }
  // Given the gradient, pick the "best fit" of the 26 neighboring points
  GRID_INDEX next_index = index;
  const double step_resolution = GetResolution() * 0.06125;
  if (working_gradient.x() > step_resolution)
  {
      next_index.x++;
  }
  else if (working_gradient.x() < -step_resolution)
  {
      next_index.x--;
  }
  if (working_gradient.y() > step_resolution)
  {
      next_index.y++;
  }
  else if (working_gradient.y() < -step_resolution)
  {
      next_index.y--;
  }
  if (working_gradient.z() > step_resolution)
  {
      next_index.z++;
  }
  else if (working_gradient.z() < -step_resolution)
  {
      next_index.z--;
  }
  return next_index;
}

VoxelGrid::VoxelGrid<Eigen::Vector3d>
SignedDistanceField::ComputeLocalExtremaMap() const
{
  VoxelGrid<Eigen::Vector3d> watershed_map(
        GetOriginTransform(), GetResolution(),
        GetXSize(), GetYSize(), GetZSize(),
        Eigen::Vector3d(-std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()));
  for (int64_t x_idx = 0; x_idx < watershed_map.GetNumXCells(); x_idx++)
  {
    for (int64_t y_idx = 0; y_idx < watershed_map.GetNumYCells(); y_idx++)
    {
      for (int64_t z_idx = 0; z_idx < watershed_map.GetNumZCells(); z_idx++)
      {
        // We use an "unsafe" function here because we know all the indices
        // we provide it will be safe
        FollowGradientsToLocalExtremaUnsafe(watershed_map, x_idx, y_idx, z_idx);
      }
    }
  }
  return watershed_map;
}

///////////////////////////////////////////////////////////////////////
// Serialization, saving, loading, etc.
///////////////////////////////////////////////////////////////////////

#ifdef ROS_EXISTS

sdf_tools::SDF SignedDistanceField::GetMessageRepresentation(
    const SignedDistanceField& sdf)
{
  sdf_tools::SDF sdf_message;
  sdf_message.header.stamp = ros::Time::now();
  sdf_message.header.frame_id = sdf.GetFrame();
  std::vector<uint8_t> buffer;
  sdf.SerializeSelf(buffer);
  sdf_message.serialized_sdf = ZlibHelpers::CompressBytes(buffer);
  sdf_message.is_compressed = true;
  return sdf_message;
}

SignedDistanceField SignedDistanceField::LoadFromMessageRepresentation(
    const sdf_tools::SDF& message)
{
  if (message.is_compressed)
  {
    const std::vector<uint8_t> uncompressed_sdf
        = ZlibHelpers::DecompressBytes(message.serialized_sdf);
    SignedDistanceField sdf;
    sdf.DeserializeSelf(uncompressed_sdf, 0);
    return sdf;
  }
  else
  {
    SignedDistanceField sdf;
    sdf.DeserializeSelf(message.serialized_sdf, 0);
    return sdf;
  }
}

visualization_msgs::Marker SignedDistanceField::ExportForDisplay(
    const float alpha) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "sdf_display";
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
  double min_distance = 0.0;
  double max_distance = 0.0;
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Update minimum/maximum distance variables
        const float distance
            = GetImmutable(x_index, y_index, z_index).first;
        if (distance < min_distance)
        {
          min_distance = distance;
        }
        if (distance > max_distance)
        {
          max_distance = distance;
        }
        // Convert SDF indices into a real-world location
        const Eigen::Vector4d location
            = GridIndexToLocationGridFrame(x_index, y_index, z_index);
        geometry_msgs::Point new_point;
        new_point.x = location(0);
        new_point.y = location(1);
        new_point.z = location(2);
        display_rep.points.push_back(new_point);
      }
    }
  }
  // Add colors for all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
      {
        // Update minimum/maximum distance variables
        const float distance = GetImmutable(x_index, y_index, z_index).first;
        std_msgs::ColorRGBA new_color;
        new_color.a = arc_helpers::ClampValue(alpha, 0.0f, 1.0f);
        if (distance > 0.0)
        {
          new_color.b = 0.0;
          new_color.g = (std::abs(distance / max_distance) * 0.8) + 0.2;
          new_color.r = 0.0;
        }
        else if (distance < 0.0)
        {
          new_color.b = 0.0;
          new_color.g = 0.0;
          new_color.r = (std::abs(distance / min_distance) * 0.8) + 0.2;
        }
        else
        {
          new_color.b = 1.0;
          new_color.g = 0.0;
          new_color.r = 0.0;
        }
        display_rep.colors.push_back(new_color);
      }
    }
  }
  return display_rep;
}

visualization_msgs::Marker SignedDistanceField::ExportForDisplayCollisionOnly(
    const float alpha) const
{
  visualization_msgs::Marker display_rep;
  // Populate the header
  display_rep.header.frame_id = frame_;
  // Populate the options
  display_rep.ns = "sdf_display";
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
  // Color it
  std_msgs::ColorRGBA collision_color;
  collision_color.a = alpha;
  collision_color.b = 0.0;
  collision_color.g = 0.0;
  collision_color.r = 1.0;
  display_rep.color = collision_color;
  // Add all the cells of the SDF to the message
  for (int64_t x_index = 0; x_index < GetNumXCells(); x_index++)
  {
      for (int64_t y_index = 0; y_index < GetNumYCells(); y_index++)
      {
          for (int64_t z_index = 0; z_index < GetNumZCells(); z_index++)
          {
              // Update minimum/maximum distance variables
              const float distance
                  = GetImmutable(x_index, y_index, z_index).first;
              if (distance <= 0.0)
              {
                  // Convert SDF indices into a real-world location
                  const Eigen::Vector4d location
                      = GridIndexToLocationGridFrame(x_index, y_index, z_index);
                  geometry_msgs::Point new_point;
                  new_point.x = location(0);
                  new_point.y = location(1);
                  new_point.z = location(2);
                  display_rep.points.push_back(new_point);
              }
          }
      }
  }
  return display_rep;
}

#endif
