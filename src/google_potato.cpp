//
// Created by Denis Koterov on 10/12/24.
//

#include "google_potato.h"

#include <absl/memory/memory.h>
#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/io/image.h>
#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/map_builder.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>