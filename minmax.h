/*
    Copyright (c) 2013, Taiga Nomi
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY 
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include <deque>
#include <iterator>

/**
 * 1D Maximum-Minimum filters
 *
 * it implements fast min-max filter algorithms described in following paper:
 * D. Lemire, "Streaming maximum-minimum filter using no more than three
 * comparisons per element", Nordic Journal of Computing 13 (4) (2006) 328-339.
 *
 * @param first, last            [in]  forward iterators defining the input range
 *                                     [first, last) to process
 * @param w                      [in]  window size of the filter
 * @param min_values, max_values [out] output iterator defining the beginning
 *                                     of the destination range. output size is
 *                                     distance(first, last) - w + 1.
 */
template<typename InputIter, typename OutputIter>
void minmax_filter(InputIter first, InputIter last, size_t w,
                   OutputIter min_values, OutputIter max_values)
{
    typedef typename std::iterator_traits<InputIter>::value_type value_t;
    typedef typename std::iterator_traits<InputIter>::reference ref_t;
    std::deque<std::pair<size_t, ref_t> > U, L;

    U.emplace_back(0, *first);
    L.emplace_back(0, *first);
    value_t prev = *first++;
    size_t i;

    for (i = 1; first != last; ++i, ++first) {
        if (i >= w) {
            *min_values++ = L.front().second;
            *max_values++ = U.front().second;
        }

        if (*first > prev) {
            U.pop_back();
            while (!U.empty() && *first > U.back().second)
                U.pop_back();
        } else {
            L.pop_back();
            while (!L.empty() && *first < L.back().second) 
                L.pop_back();
        }
        U.emplace_back(i, *first);
        L.emplace_back(i, *first);

        if (i == w + U.front().first)
            U.pop_front();
        else if (i == w + L.front().first)
            L.pop_front();

        prev = *first;
    }

    if (i >= w) {
        *max_values = U.front().second;
        *min_values = L.front().second;
    }
}

//--------------------------------------------------
// test code
#ifdef MINMAX_TESTING

#include "picotest.h"
#include <random>

namespace testing {

namespace naive {
/**
 * naive implementations of min-max filter
 */
template<typename InputIter, typename OutputIter>
void minmax_filter(InputIter first, InputIter last, size_t w,
                   OutputIter min_values, OutputIter max_values)
{
    typedef typename std::iterator_traits<InputIter>::value_type value_t;
    size_t size = std::distance(first, last);

    for (size_t i = 0; i < size - w + 1; i++, ++first, ++min_values, ++max_values) {
        value_t min_value, max_value;
        min_value = max_value = *first;

        for (size_t j = 1; j < w; j++) {
            min_value = std::min(min_value, first[j]);
            max_value = std::max(max_value, first[j]);
        }
        *min_values = min_value;
        *max_values = max_value;
    }
}
} // namespace naive


TEST(single_element, minmax_double) {
    double v[2] = { 3, 0 };
    double minV, maxV;

    minmax_filter(v, v + 1, 1, &minV, &maxV);
    ASSERT_EQ(3, minV);
    ASSERT_EQ(3, maxV);
}

TEST(large_window, minmax_double) {
    double v[3] = { 3, 1, 5 };
    double minV = -1;
    double maxV = -1;

    minmax_filter(v, v + 3, 5, &minV, &maxV);
    ASSERT_EQ(minV, -1); // no update
    ASSERT_EQ(maxV, -1); // no update

    minmax_filter(v, v + 3, 3, &minV, &maxV);
    ASSERT_EQ(minV, 1);
    ASSERT_EQ(maxV, 5);
}

TEST(normal_seq, minmax_double) {
    double v[10]         = { 8, 0, 3, 2, 5, 0, 0, 2, 1, -9 };
    double min3_expected[8] = { 0, 0, 2, 0, 0, 0, 0,-9 };
    double max3_expected[8] = { 8, 3, 5, 5, 5, 2, 2, 2 };
    double min5_expected[6] =    { 0, 0, 0, 0, 0,-9 };
    double max5_expected[6] =    { 8, 5, 5, 5, 5, 2 };

    double minV[8], maxV[8];

    minmax_filter(v, v + 10, 3, minV, maxV);

    for (int i = 0; i < 8; i++) {
        ASSERT_EQ(minV[i], min3_expected[i]);
        ASSERT_EQ(maxV[i], max3_expected[i]);
    }

    minmax_filter(v, v + 10, 5, minV, maxV);

    for (int i = 0; i < 6; i++) {
        ASSERT_EQ(minV[i], min5_expected[i]);
        ASSERT_EQ(maxV[i], max5_expected[i]);
    }
}

TEST(random, minmax_double) {
    std::vector<double> v;
    std::vector<double> minV, maxV, minV_naive, maxV_naive;

    const int size = 10000;
    std::mt19937 engine;
    std::uniform_real_distribution<> dist;

    for (int i = 0; i < size; i++) {
        v.push_back(dist(engine));
    }
    
    minmax_filter(v.begin(), v.end(), 3,
        std::back_inserter(minV), std::back_inserter(maxV));

    naive::minmax_filter(v.begin(), v.end(), 3,
        std::back_inserter(minV_naive), std::back_inserter(maxV_naive));

    ASSERT_EQ(minV, minV_naive);
    ASSERT_EQ(maxV, maxV_naive);
}

} // namespace testing

#endif // MINMAX_TESTING
