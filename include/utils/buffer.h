// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <alessandro.fornasier@ieee.org>

#ifndef BUFFER_H
#define BUFFER_H

#include <Eigen/Eigen>

namespace Buffer
{
/**
 * @brief The Buffer struct is a interface for estimate buffers.
 * It only contains the timestamp and implements operators for comparision
 * reasons
 */
struct Buffer
{
  double timestamp;

  /// Sort function to allow for using of STL containers
  bool operator<(const Buffer& other) const
  {
    return timestamp < other.timestamp;
  }
};

/**
 * @brief Position buffer struct
 */
struct positionBuffer : Buffer
{
  Eigen::Vector3d p;
};

}  // namespace Buffer

#endif  // BUFFER_H
