/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Nathan Brooks
   Desc: Manage license for TrackSuite
*/

#pragma once

// TrackJoint
#include "trackjoint/LexActivator.h"

// C++
#include <string>

namespace trackjoint
{
class LicenseManager
{
public:
  LicenseManager();

  bool CheckLicenseStatus();
  void Parse(int argc, char* argv[]);

private:
  void SetProductInfo();
  void DeactivateTrackSuiteLicense();
  void ActivateTrackSuiteLicense(std::string key);
  void ActivateTrackSuiteTrial(std::string key);
  void PrintVersion();
};  // end class LicenseManager
}  // namespace trackjoint
