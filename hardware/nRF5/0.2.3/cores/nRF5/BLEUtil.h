// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef _BLE_UTIL_H_
#define _BLE_UTIL_H_

#include "Stream.h"
class BLEUtil
{
  public:
    static void addressToString(const unsigned char *in, char* out);

    static void printBuffer(Stream *stream, const unsigned char* buffer, unsigned char length);
};

#endif
