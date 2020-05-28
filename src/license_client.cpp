/*********************************************************************
 * Copyright (c) 2019, PickNik Consulting
 * All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

#include "trackjoint/license_manager.h"

int main(int argc, char* argv[])
{
  trackjoint::LicenseManager license_manager;
  license_manager.Parse(argc, argv);

  return 0;
}
