/*
 * Copyright (c) 2026, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "zlib_cpp.hpp"

namespace
{

// Build a deterministic byte buffer resembling packed PointCloud2 data.
std::vector<uint8_t> makeCloudBytes(std::size_t n)
{
  std::vector<uint8_t> bytes(n);
  for (std::size_t i = 0; i < n; ++i) {
    bytes[i] = static_cast<uint8_t>((i * 31u + 7u) & 0xFFu);
  }
  return bytes;
}

// Compress through the zlib::Comp wrapper and flatten the chunk list.
std::vector<uint8_t> compress(const std::vector<uint8_t> & in)
{
  zlib::Comp comp(zlib::Comp::Level::Default, true);
  EXPECT_TRUE(comp.IsSucc());
  const auto chunks = comp.Process(in.data(), in.size(), true);
  std::vector<uint8_t> out;
  for (const auto & chunk : chunks) {
    out.insert(out.end(), chunk->ptr, chunk->ptr + chunk->size);
  }
  return out;
}

// Decompress through the zlib::Decomp wrapper.
std::vector<uint8_t> decompress(const std::vector<uint8_t> & in)
{
  zlib::Decomp decomp;
  auto block = zlib::AllocateData(in.size());
  std::memcpy(block->ptr, in.data(), in.size());
  const auto chunks = decomp.Process(block);
  const auto expanded = zlib::ExpandDataList(chunks);
  return std::vector<uint8_t>(expanded->ptr, expanded->ptr + expanded->size);
}

}  // namespace

// Data must survive a compress -> decompress cycle byte-for-byte, including
// buffers larger than the wrapper's internal chunk size and the empty edge case.
TEST(ZlibRoundTrip, PreservesDataAcrossSizes)
{
  const std::vector<std::size_t> sizes = {0, 1, 100, 4096, 200000};
  for (const std::size_t n : sizes) {
    const auto original = makeCloudBytes(n);
    const auto result = decompress(compress(original));
    EXPECT_EQ(result, original) << "round-trip failed for " << n << " bytes";
  }
}

TEST(ZlibRoundTrip, CompressorInitializes)
{
  zlib::Comp comp(zlib::Comp::Level::Default, true);
  EXPECT_TRUE(comp.IsSucc());
}

// Sanity that compression actually runs: repetitive input must shrink.
TEST(ZlibRoundTrip, ShrinksCompressibleData)
{
  const std::vector<uint8_t> zeros(100000, 0);
  EXPECT_LT(compress(zeros).size(), zeros.size());
}
