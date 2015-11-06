// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <list>
#include <ctype.h>
#include <stddef.h>
#include <thread>
#include <inttypes.h>
#include <vector>
#include <mutex>

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "update_protocol.pb.h"

#include "manymouse.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
#include <google/protobuf/arena.h>
#include "../Include/NatNetTypes.h"
#include "../Include/NatNetClient.h"

#include "wiimote.h"

#include "update_protocol.pb.h"

// TODO: reference additional headers your program requires here
