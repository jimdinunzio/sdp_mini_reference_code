/*
 * SlamTec Base Ref Design
 * Copyright 2009 - 2017 RoboPeak
 * Copyright 2013 - 2017 Shanghai SlamTec Co., Ltd.
 * http://www.slamtec.com
 * All rights reserved.
 */
/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "common/common.h"
#include "utils/filters.h"

/*
 * 平均滤波函数
 * 将新数据放入滤波器再求平均
 * 数据类型：unsigned short
 */
_u16 add_to_avg_filter_u16(_u16 new_data, _u16 * filter, size_t pos, size_t size)
{
    _s32 * sum = (_s32 *)(filter + size);

    *sum  -= (_s32)filter[pos];
    *sum  += (_s32)new_data;
    filter[pos] = new_data;

    return (_u16)((*sum)/size);
}

/*
 * 平均滤波函数
 * 将新数据放入滤波器再求平均
 * 数据类型：short
 */
_s16 add_to_avg_filter_s16(_s16 new_data, _s16 * filter, size_t pos, size_t size)
{
    _s32 * sum = (_s32 *)(filter + size);

    *sum  -= (_s32)filter[pos];
    *sum  += (_s32)new_data;
    filter[pos] = new_data;

    return (_s16)((*sum)/((_s32)size));
}

/*
 * 平均滤波函数
 * 将新数据放入滤波器再求平均
 * 数据类型：unsigned int
 */
_u32 add_to_avg_filter_u32(_u32 new_data, _u32 * filter, size_t pos, size_t size)
{
    _s64 * sum = (_s64 *)(filter + size);

    *sum  -= (_s64)filter[pos];
    *sum  += (_s64)new_data;
    filter[pos] = new_data;

    return (_u32)((*sum)/size);
}