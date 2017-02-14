/*
 * Copyright (C) 2017 Intel Deutschland GmbH, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdint.h>

#pragma pack(push, 1)

struct BUILD_INFO
{
	uint16_t version_major;
	uint16_t version_minor;
	uint32_t build_date;
	uint32_t build_number;
	uint32_t configuration;
	uint16_t svn_revision;
	uint8_t svn_modified;
	int8_t svn_url[64];
};

#pragma pack(pop)

extern struct BUILD_INFO buildInfo;

void generateBuildInfo(void);
