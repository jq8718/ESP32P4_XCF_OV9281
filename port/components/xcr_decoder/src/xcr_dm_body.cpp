/* Current frame recovery using the DataMatrix finder itself. The detector
 * finds/samples the symbol; the project's ECC200 and corner fit stay in C. */
#include "BitMatrix.h"
#include "datamatrix/DMDetector.h"
#include "PerspectiveTransform.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include "xcr_dm_body.h"
#include <vector>

extern "C" {
#include "datamatrix.h"
#include "xcr_port.h"
}

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32P4
extern "C" void xcr_dm_vertical_pie(const uint8_t*, const uint8_t*, const uint8_t*,
                                   uint8_t*, uint8_t*, int);
extern "C" void xcr_dm_classify_pie(const uint8_t*, const uint8_t*, const uint8_t*,
                                   uint8_t*, int, const uint8_t*);
extern "C" void xcr_dm_sample2_pie(const uint8_t*, const uint8_t*, const uint8_t*,
                                  uint8_t*, uint8_t*, uint8_t*, int);
extern "C" void xcr_dm_sample_row2_pie(const uint8_t*, uint8_t*, int);
#endif

/* A spaced 3x3 stencil adapts to dark paper next to a bright edge.
 * Separable extrema require only row scratch; the original Gray8
 * remains available for subpixel refinement. */
template<int Step>
static bool threshold(const Image& image, ZXing::BitMatrix& bits, const XcrDeadline* deadline)
{
    constexpr int radius = 5;
    static constexpr int bias = Step <= 2 ? 6 : 0;
    constexpr uint8_t constants[] = {8, bias};
#ifdef CONFIG_IDF_TARGET_ESP32P4
    alignas(16) static constexpr auto vector_constants = [] {
        std::array<uint8_t, 32> values{};
        std::fill_n(values.begin(), 16, 8);
        std::fill_n(values.begin() + 16, 16, bias);
        return values;
    }();
#endif
    const int width = bits.width(), height = bits.height();
    const int pitch = (width + 15) & ~15;
    // The stencil reuses a source row at y-5, y and y+5. Cache its sampled
    // pixels so every Gray8 row crosses PSRAM only once, also during dual-core IO.
    constexpr int cached_rows = 2 * radius + 1;
    std::vector<uint8_t> storage((cached_rows + 3) * pitch + 15);
    auto* rows = reinterpret_cast<uint8_t*>((reinterpret_cast<uintptr_t>(storage.data()) + 15) & ~uintptr_t(15));
    std::array<int, cached_rows> row_ids;
    row_ids.fill(-1);
    auto* lo = rows + cached_rows * pitch;
    auto* hi = lo + pitch;
    auto* mask = hi + pitch;
    auto sample_row = [&](int y) {
        int index = y % cached_rows;
        auto* sampled_row = rows + index * pitch;
        if (row_ids[index] == y) return sampled_row;
        const auto* source = image.pixels + static_cast<size_t>(y * Step) * image.width;
        int sampled = 0;
#ifdef CONFIG_IDF_TARGET_ESP32P4
        if constexpr (Step == 2) {
            if (!(reinterpret_cast<uintptr_t>(source) & 15) && !(image.width & 15)) {
                sampled = width & ~15;
                if (sampled)
                    xcr_dm_sample_row2_pie(source, sampled_row, sampled);
            }
        }
#endif
        for (int x = sampled; x < width; ++x)
            sampled_row[x] = source[x * Step];
        row_ids[index] = y;
        return sampled_row;
    };
    for (int y = 0; y < height; ++y) {
        if ((y & 7) == 0 && xcr_deadline_expired(deadline))
            return false;
        const auto* gray = sample_row(y);
        const auto* top_sample = sample_row(y < radius ? y : y - radius);
        const auto* bottom_sample = sample_row(y + radius < height ? y + radius : y);
#ifdef CONFIG_IDF_TARGET_ESP32P4
        xcr_dm_vertical_pie(top_sample, gray, bottom_sample, lo, hi, pitch);
#else
        for (int x = 0; x < width; ++x) {
            lo[x] = std::min({top_sample[x], gray[x], bottom_sample[x]});
            hi[x] = std::max({top_sample[x], gray[x], bottom_sample[x]});
        }
#endif
        auto classify = [&](int x) {
            int left = x < radius ? x : x - radius;
            int right = x + radius < width ? x + radius : x;
            int l = std::min({lo[left], lo[x], lo[right]});
            int h = std::max({hi[left], hi[x], hi[right]});
            mask[x] = h - l > constants[0] && 2 * gray[x] + constants[1] < l + h ? ZXing::BitMatrix::SET_V : 0;
        };
        int x = 0;
#ifdef CONFIG_IDF_TARGET_ESP32P4
        static_assert(radius == 5);
        int inner_end = (width - radius) & ~15;
        if (inner_end > 16) {
            for (; x < 16; ++x)
                classify(x);
            xcr_dm_classify_pie(lo, hi, gray + 16, mask + 16, inner_end - 16, vector_constants.data());
            x = inner_end;
        }
#endif
        for (; x < width; ++x)
            classify(x);
        std::memcpy(bits.row(y).begin(), mask, width);
    }
    return true;
}

extern "C" void* xcr_dm_body_prepare(const Image* image, int step,
    const XcrDeadline* deadline, uint64_t* threshold_us)
{
    if (!image || !image->pixels || (step != 2 && step != 3) || xcr_deadline_expired(deadline))
        return nullptr;
    try {
        auto bits = std::make_unique<ZXing::BitMatrix>(
            (image->width + step - 1) / step, (image->height + step - 1) / step);
        const auto start = xcr_port_time_us();
        bool complete = step == 3 ? threshold<3>(*image, *bits, deadline) : threshold<2>(*image, *bits, deadline);
        *threshold_us += xcr_port_time_us() - start;
        return complete ? bits.release() : nullptr;
    } catch (const std::exception&) {
        return nullptr;
    }
}

extern "C" void xcr_dm_body_discard(void* state)
{
    delete static_cast<ZXing::BitMatrix*>(state);
}

static float gray_at(const Image& image, ZXing::PointF p)
{
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        p.x < 0 || p.y < 0 || p.x >= image.width - 1 || p.y >= image.height - 1)
        return -1;
    int x = static_cast<int>(p.x), y = static_cast<int>(p.y);
    float dx = p.x - x, dy = p.y - y;
    const auto* row = image.pixels + static_cast<size_t>(y) * image.width + x;
    return (1 - dy) * ((1 - dx) * row[0] + dx * row[1]) +
        dy * ((1 - dx) * row[image.width] + dx * row[image.width + 1]);
}

/* Reconsider only a detected current-frame quadrilateral. Size controls the
 * sampling footprint; measured paper variation controls the threshold order.
 * No image or decision from an earlier frame enters this function. */
static int decode_gray_candidate(const Image& image, BarcodeResult& result,
    const ZXing::QuadrilateralI& position, int step, const XcrDeadline* deadline,
    XcrDmBodyTrace* trace)
{
    ZXing::QuadrilateralF quad;
    float shortest = INFINITY, longest = 0;
    for (int p = 0; p < 4; ++p)
        quad[p] = step * ZXing::PointF(position[p]);
    for (int p = 0; p < 4; ++p) {
        float side = ZXing::distance(quad[p], quad[(p + 1) & 3]);
        shortest = std::min(shortest, side);
        longest = std::max(longest, side);
    }
    if (shortest < 20 || longest > 2 * shortest || !ZXing::IsConvex(quad))
        return 0;
    ZXing::PerspectiveTransform map(ZXing::Rectangle<ZXing::PointF>(10, 10, 0), quad);
    if (!map.isValid())
        return 0;
    // Below ~4.5 px/module, averaging neighbours can cross a module edge.
    const int taps = shortest < 45 ? 1 : 5;
    constexpr float phases[5][2] = {{0, 0}, {-.12f, 0}, {.12f, 0}, {0, -.12f}, {0, .12f}};
    for (auto phase : phases) {
        if (xcr_deadline_expired(deadline))
            break;
        uint8_t gray[100], modules[100], paper[9] = {};
        unsigned histogram[256] = {}, sum = 0;
        int minimum = 255, maximum = 0;
        for (int y = 0; y < 10; ++y) {
            for (int x = 0; x < 10; ++x) {
                float value = 0;
                for (int tap = 0; tap < taps; ++tap) {
                    float dx = tap == 1 ? -.12f : tap == 2 ? .12f : 0;
                    float dy = tap == 3 ? -.12f : tap == 4 ? .12f : 0;
                    float g = gray_at(image, map({x + .5f + phase[0] + dx, y + .5f + phase[1] + dy}));
                    if (g < 0) return 0;
                    value += g;
                }
                int g = static_cast<int>(value / taps + .5f);
                gray[(9 - y) * 10 + x] = g;
                histogram[g]++;
                sum += g;
                minimum = std::min(minimum, g);
                maximum = std::max(maximum, g);
                auto& bright = paper[std::min(y / 4, 2) * 3 + std::min(x / 4, 2)];
                bright = std::max(bright, static_cast<uint8_t>(g));
            }
        }
        if (maximum - minimum < 9)
            return 0;
        std::sort(std::begin(paper), std::end(paper));
        // Ignore the darkest two patches, which may contain only black cells.
        bool shadow = paper[8] - paper[2] > std::max(16, (maximum - minimum) / 4);
        unsigned dark_count = 0, dark_sum = 0;
        float best = -1;
        int otsu = (minimum + maximum) / 2;
        for (int g = minimum; g < maximum; ++g) {
            dark_count += histogram[g];
            dark_sum += g * histogram[g];
            if (!dark_count || dark_count == 100) continue;
            float delta = static_cast<float>(dark_sum) / dark_count -
                static_cast<float>(sum - dark_sum) / (100 - dark_count);
            float score = dark_count * (100 - dark_count) * delta * delta;
            if (score > best) { best = score; otsu = g; }
        }
        for (int attempt = 0; attempt < 2 && !xcr_deadline_expired(deadline); ++attempt) {
            bool local = attempt == 0 ? shadow : !shadow;
            if (trace) {
                trace->gray_attempts++;
                trace->binarizer = local ? 2 : 1;
            }
            for (int y = 0; y < 10; ++y) {
                for (int x = 0; x < 10; ++x) {
                    int cutoff = otsu;
                    if (local) {
                        int lo = 255, hi = 0;
                        for (int j = std::max(0, y - 2); j <= std::min(9, y + 2); ++j)
                            for (int i = std::max(0, x - 2); i <= std::min(9, x + 2); ++i) {
                                lo = std::min(lo, int(gray[j * 10 + i]));
                                hi = std::max(hi, int(gray[j * 10 + i]));
                            }
                        cutoff = (lo + hi) / 2;
                    }
                    modules[y * 10 + x] = gray[y * 10 + x] <= cutoff;
                }
            }
            if (int length = datamatrix_decode_grid10(modules, &result); length > 0)
                return length;
        }
    }
    return 0;
}

static int body_finish(const Image* image, BarcodeResult* result, int step,
    void* state, const XcrDeadline* deadline, XcrDmBodyTrace* trace, int gray_limit)
{
    if (!state || !image || !result || xcr_deadline_expired(deadline))
        return 0;
    try {
        const auto& bits = *static_cast<ZXing::BitMatrix*>(state);
        // Detect is a lazy coroutine. Keep the std::function alive until its
        // generator is destroyed; an implicit temporary from a lambda dies
        // at the range initializer and leaves the coroutine with a dangling
        // reference as soon as it resumes after yielding a candidate.
        std::function<bool()> cancelled = [deadline]() { return xcr_deadline_expired(deadline); };
        int gray_candidates = 0;
        for (auto&& found : ZXing::DataMatrix::Detect(bits, true, true, false, cancelled)) {
            if (trace) {
                trace->detector_candidates++;
                float shortest_squared = INFINITY;
                for (int p = 0; p < 4; ++p) {
                    auto delta = found.position()[p] - found.position()[(p + 1) & 3];
                    shortest_squared = std::min(shortest_squared, float(ZXing::dot(delta, delta)));
                }
                trace->candidate_px = step * std::sqrt(shortest_squared);
                trace->binarizer = 0;
            }
            const auto& grid = found.bits();
            uint8_t modules[100] = {};
            /* ZXing keeps the solid borders at left and bottom; the C
             * grid decoder keeps them at left and top. */
            BarcodeResult candidate{};
            int length = 0;
            if (grid.width() == 10 && grid.height() == 10) {
                for (int y = 0; y < 10; ++y)
                    for (int x = 0; x < 10; ++x)
                        modules[y * 10 + x] = grid.get(x, 9 - y);
                length = datamatrix_decode_grid10(modules, &candidate);
            }
            if (length <= 0 && gray_candidates++ < gray_limit)
                length = decode_gray_candidate(*image, candidate, found.position(), step, deadline, trace);
            if (length <= 0)
                continue;
            for (int p = 0; p < 4; ++p) {
                candidate.corners[2 * p] = std::clamp(found.position()[p].x * step, 0, image->width - 1);
                candidate.corners[2 * p + 1] = std::clamp(found.position()[p].y * step, 0, image->height - 1);
                candidate.geometry_seed[2 * p] = candidate.corners[2 * p];
                candidate.geometry_seed[2 * p + 1] = candidate.corners[2 * p + 1];
            }
            candidate.geometry_valid = candidate.geometry_seed_valid = 1;
            *result = candidate;
            return length;
        }
        if (trace) trace->search_complete = !xcr_deadline_expired(deadline);
    } catch (const std::exception&) {
        /* The fast path can still run if scratch allocation fails. */
    }
    return 0;
}

extern "C" int xcr_dm_body_finish(const Image* image, BarcodeResult* result, int step,
    void* state, const XcrDeadline* deadline, XcrDmBodyTrace* trace)
{
    return body_finish(image, result, step, state, deadline, trace, 3);
}

static void threshold_otsu(const std::vector<uint8_t>& pixels, ZXing::BitMatrix& bits)
{
    unsigned histogram[256] = {}, total = pixels.size(), sum = 0;
    for (uint8_t v : pixels) { histogram[v]++; sum += v; }
    unsigned n = 0, dark_sum = 0;
    float best = -1;
    int cutoff = 128;
    for (int v = 0; v < 255; ++v) {
        n += histogram[v]; dark_sum += v * histogram[v];
        if (!n || n == total) continue;
        float delta = float(dark_sum) / n - float(sum - dark_sum) / (total - n);
        float score = float(n) * (total - n) * delta * delta;
        if (score > best) { best = score; cutoff = v; }
    }
    for (int y = 0; y < bits.height(); ++y) {
        auto* row = bits.row(y).begin();
        for (int x = 0; x < bits.width(); ++x)
            row[x] = pixels[y * bits.width() + x] <= cutoff ? ZXing::BitMatrix::SET_V : 0;
    }
}

extern "C" int xcr_dm_body_search_traced(const Image* image, BarcodeResult* result,
    const void* state, const XcrDeadline* deadline, int* sample_step, XcrDmBodyTrace* trace)
{
    if (!image || !result || !state || xcr_deadline_expired(deadline))
        return 0;
    try {
        const auto& bits = *static_cast<const ZXing::BitMatrix*>(state);
        struct Hint { int x, y, size, score; };
        std::array<Hint, 12> hints{};
        const int w = bits.width(), h = bits.height();
        auto rank_current_hint = [&](Hint hint) {
            int quiet_min = 12;
            int radius = hint.size / 2 + 2;
            if (hint.x < radius || hint.y < radius || hint.x + radius >= w || hint.y + radius >= h)
                return;
            for (int side = 0; side < 4; ++side) {
                int quiet = 0;
                for (int i = 0; i < 12; ++i) {
                    int offset = (2 * i - 11) * hint.size / 24;
                    int x = hint.x + (side < 2 ? (side ? radius : -radius) : offset);
                    int y = hint.y + (side >= 2 ? (side == 3 ? radius : -radius) : offset);
                    quiet += !bits.get(x, y);
                }
                quiet_min = std::min(quiet_min, quiet);
            }
#ifndef ESP_PLATFORM
            if (std::getenv("XCR_SMALL_TRACE"))
                std::fprintf(stderr, "quiet %d %d size=%d min=%d\n", hint.x, hint.y, hint.size, quiet_min);
#endif
            if (quiet_min < 6) return;
            // The label's small external dots distinguish it from L-like
            // desk/monitor edges. Rank, rather than require, those dots.
            int dots = 0;
            for (int dy : {-1, 1})
                for (int dx : {-1, 1}) {
                    bool matched = false;
                    int cx = hint.x + dx * (3 * hint.size / 4);
                    int cy = hint.y + dy * (3 * hint.size / 4);
                    for (int y = std::max(3, cy - 4); y <= std::min(h - 4, cy + 4) && !matched; ++y)
                        for (int x = std::max(3, cx - 4); x <= std::min(w - 4, cx + 4); ++x) {
                            if (!bits.get(x, y) || !bits.get(x + 1, y) ||
                                !bits.get(x, y + 1) || !bits.get(x + 1, y + 1))
                                continue;
                            int r = 2;
                            if (x < r || y < r || x + r + 1 >= w || y + r + 1 >= h) continue;
                            int quiet = !bits.get(x - r, y) + !bits.get(x + r + 1, y) +
                                        !bits.get(x, y - r) + !bits.get(x, y + r + 1) +
                                        !bits.get(x - r, y - r) + !bits.get(x + r + 1, y + r + 1);
                            if (quiet >= 5) { matched = true; break; }
                        }
                    dots += matched;
                }
            hint.score += dots * 12;
            auto* worst = &*std::min_element(hints.begin(), hints.end(),
                                            [](Hint a, Hint b) { return a.score < b.score; });
            for (auto& previous : hints) {
                if (previous.score && std::abs(hint.x - previous.x) < 8 &&
                    std::abs(hint.y - previous.y) < 8) {
                    worst = &previous;
                    break;
                }
            }
            if (hint.score > worst->score)
                *worst = hint;
        };
        auto inspect = [&](int left, int right, int y) {
            int span = right - left;
            if (span < 10 || span > 20 || left < 3 || right + 3 >= w)
                return;
            for (int x : {left, right - 1}) {
                int outside = x == left ? -1 : 1;
                for (int dy : {-1, 1}) {
                    if (bits.get(x, y - 3 * dy) || y + span * dy < 0 || y + span * dy >= h)
                        continue;
                    int strength = 0, background = 0;
                    for (int j : {2, span / 3, span / 2, 3 * span / 4, span - 2}) {
                        const auto* row = bits.row(y + j * dy).begin();
                        strength += row[x] ? 2 : (row[x - 1] || row[x + 1] ? 1 : 0);
                        background += !row[x + 3 * outside];
                    }
                    if (strength < 7 || background < 4)
                        continue;
                    const auto* outer = bits.row(y - 2 * dy).begin();
                    int quiet = !outer[left + span / 4] + !outer[left + span / 2] +
                                !outer[left + 3 * span / 4];
                    int transitions = 0;
                    for (int j : {span / 3, span / 2, 2 * span / 3}) {
                        const auto* row = bits.row(y + j * dy).begin();
                        int changes = 0;
                        for (int k = left + 3; k < right - 2; ++k)
                            changes += row[k] != row[k - 1];
                        transitions += std::min(changes, 6);
                    }
                    // At ~24 px a 2px map merges alternating modules. Keep
                    // its strong solid L even when only 3 transitions remain;
                    // full-resolution detection and ECC still validate it.
                    if (quiet >= 2 && transitions >= (span < 14 ? 3 : 6))
                        rank_current_hint({left + span / 2, y + (span / 2) * dy, span,
                                  2 * strength + 3 * background + 3 * quiet + 2 * transitions});
                }
            }
        };
        // Preserve the two solid legs as a cheap hint even when the
        // alternating modules are undersampled. Scan the current map only.
        for (int y = 4; y + 4 < h; ++y) {
            if ((y & 15) == 4 && xcr_deadline_expired(deadline))
                break;
            const auto* row = bits.row(y).begin();
            for (int x = 0; x < w;) {
                while (x < w && !row[x]) ++x;
                int left = x;
                while (x < w && (row[x] || (x + 1 < w && row[x + 1]))) ++x;
                if (x != left)
                    inspect(left, x, y);
            }
        }
        std::sort(hints.begin(), hints.end(), [](Hint a, Hint b) { return a.score > b.score; });
        // Decide from this frame's short L candidates only. A confident tiny
        // candidate gets four cheap local trials; otherwise search the 2px map
        // before trying background textures at full pixel resolution.
        const bool tiny_first = hints[0].score >= 90;
        if (trace) trace->hint_score = hints[0].score;
        XcrDmBodyTrace ordinary_trace{};
        auto ordinary_search = [&]() {
            XcrDeadline quick = deadline ? *deadline : XcrDeadline{};
            uint64_t limit = xcr_port_time_us() + 12000;
            if (!quick.deadline_us || quick.deadline_us > limit) quick.deadline_us = limit;
            int length = body_finish(image, result, 2, const_cast<void*>(state), &quick, &ordinary_trace, 3);
            if (trace) {
                trace->detector_candidates = ordinary_trace.detector_candidates;
                trace->search_complete = ordinary_trace.search_complete;
                trace->candidate_px = ordinary_trace.candidate_px;
                trace->gray_attempts = ordinary_trace.gray_attempts;
                trace->binarizer = ordinary_trace.binarizer;
            }
            if (length > 0 && sample_step) *sample_step = 2;
            return length;
        };
        if (!tiny_first) {
            int length = ordinary_search();
            if (length > 0) return length;
            // Neither a plausible short L nor a detected quadrilateral in
            // this frame: do not turn desk edges into full-resolution retries.
            if (hints[0].score < 90 && !ordinary_trace.detector_candidates) {
                if (trace) trace->weak_frame = 1;
                return 0;
            }
        }
        // Give each candidate a cheap first pass before spending time on
        // interpolation for a false L from the textured background.
        for (int pass = 0; pass < 2 && !xcr_deadline_expired(deadline); ++pass) {
        if (pass && tiny_first) {
            int length = ordinary_search();
            if (length > 0) return length;
        }
        for (const auto& hint : hints) {
            if (!pass && &hint == hints.data() + 4)
                break;
#ifndef ESP_PLATFORM
            if (std::getenv("XCR_SMALL_TRACE"))
                std::fprintf(stderr, "hint %d %d size=%d score=%d\n", hint.x, hint.y, hint.size, hint.score);
#endif
            if (!hint.score || xcr_deadline_expired(deadline))
                break;
            XcrDeadline candidate_deadline = deadline ? *deadline : XcrDeadline{};
            // Interpolation and gray-grid validation need more than 4 ms on
            // P4 under CSI/PSRAM contention; an early timeout costs a full retry.
            uint64_t candidate_limit = xcr_port_time_us() + (hint.size < 14 ? 8000 : pass ? 6000 : 2000);
            if (!candidate_deadline.deadline_us || candidate_deadline.deadline_us > candidate_limit)
                candidate_deadline.deadline_us = candidate_limit;
            const XcrDeadline* crop_deadline = &candidate_deadline;
            int half = hint.size + 15;
            int left = std::max(0, 2 * hint.x - half), top = std::max(0, 2 * hint.y - half);
            int width = std::min(image->width, 2 * hint.x + half) - left;
            int height = std::min(image->height, 2 * hint.y + half) - top;
            std::vector<uint8_t> pixels(size_t(width) * height);
            for (int y = 0; y < height; ++y)
                std::memcpy(pixels.data() + y * width, image->pixels + (top + y) * image->width + left, width);
            Image crop{};
            crop.pixels = pixels.data();
            crop.width = width;
            crop.height = height;
            ZXing::BitMatrix full(width, height);
            BarcodeResult candidate{};
            int length = 0;
            if (!pass && hint.size >= 14) {
                if (!threshold<1>(crop, full, crop_deadline)) continue;
                length = body_finish(&crop, &candidate, 1, &full, crop_deadline, nullptr, 0);
            }
            if (pass && !xcr_deadline_expired(crop_deadline)) {
                // Small shaded labels may have enough contrast locally but
                // too little for the background-aware stencil. Threshold only
                // this hint and retain every original pixel.
                threshold_otsu(pixels, full);
                length = xcr_dm_body_finish(&crop, &candidate, 1, &full, crop_deadline, nullptr);
            }
            int scale = 1;
            if ((pass || hint.size < 14) && length <= 0 && !xcr_deadline_expired(crop_deadline)) {
                // Interpolate only a small candidate. This supplies the
                // line tracer with subpixel phase choices on 2-3 px modules.
                std::vector<uint8_t> enlarged(size_t(width) * height * 4);
                for (int y = 0; y < 2 * height; ++y)
                    for (int x = 0; x < 2 * width; ++x) {
                        int x0 = x / 2, x1 = std::min(x0 + 1, width - 1);
                        int y0 = y / 2, y1 = std::min(y0 + 1, height - 1);
                        int a = pixels[y0 * width + x0], b = pixels[y0 * width + x1];
                        int c = pixels[y1 * width + x0], d = pixels[y1 * width + x1];
                        enlarged[y * 2 * width + x] = ((2 - (y & 1)) * ((2 - (x & 1)) * a + (x & 1) * b) +
                            (y & 1) * ((2 - (x & 1)) * c + (x & 1) * d) + 2) / 4;
                    }
                Image expanded{};
                expanded.pixels = enlarged.data();
                expanded.width = 2 * width;
                expanded.height = 2 * height;
                ZXing::BitMatrix smooth(2 * width, 2 * height);
                if (threshold<1>(expanded, smooth, crop_deadline)) {
                    length = xcr_dm_body_finish(&expanded, &candidate, 1, &smooth, crop_deadline, nullptr);
                    scale = 2;
                    if (length <= 0 && !xcr_deadline_expired(crop_deadline)) {
                        threshold_otsu(enlarged, smooth);
                        length = xcr_dm_body_finish(&expanded, &candidate, 1, &smooth, crop_deadline, nullptr);
                    }
                }
            }
            if (length <= 0)
                continue;
#ifndef ESP_PLATFORM
            if (std::getenv("XCR_SMALL_TRACE"))
                std::fprintf(stderr, "recovered pass=%d scale=%d\n", pass, scale);
#endif
            for (int p = 0; p < 4; ++p) {
                candidate.corners[2 * p] = (candidate.corners[2 * p] + scale / 2) / scale + left;
                candidate.corners[2 * p + 1] = (candidate.corners[2 * p + 1] + scale / 2) / scale + top;
                candidate.geometry_seed[2 * p] = candidate.corners[2 * p];
                candidate.geometry_seed[2 * p + 1] = candidate.corners[2 * p + 1];
            }
            *result = candidate;
            if (sample_step) *sample_step = 1;
            return length;
        }
        }
    } catch (const std::exception&) {
        // The normal detector still runs if there is no memory for a crop.
    }
    return 0;
}

extern "C" int xcr_dm_body_search(const Image* image, BarcodeResult* result,
    const void* state, const XcrDeadline* deadline, int* sample_step)
{
    return xcr_dm_body_search_traced(image, result, state, deadline, sample_step, nullptr);
}

extern "C" int xcr_dm_body_decode(const Image* image, BarcodeResult* result, int step,
    const XcrDeadline* deadline, uint64_t* threshold_us)
{
    void* state = xcr_dm_body_prepare(image, step, deadline, threshold_us);
    int decoded = xcr_dm_body_finish(image, result, step, state, deadline, nullptr);
    xcr_dm_body_discard(state);
    return decoded;
}
