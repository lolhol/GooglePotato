//
// Created by Denis Koterov on 10/12/24.
//

#ifndef GOOGLE_POTATO_H
#define GOOGLE_POTATO_H

#include <functional>

using PoseUpdateCallback = std::function<void(const char &)>;

class GooglePotato {
public:
  GooglePotato(const char *dir, const char *file);
};

#endif // GOOGLE_POTATO_H
