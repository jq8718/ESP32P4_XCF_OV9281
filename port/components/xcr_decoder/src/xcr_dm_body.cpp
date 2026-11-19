/* Current frame recovery using the DataMatrix finder itself. The detector
 * finds/samples the symbol; the project's ECC200 and corner fit stay in C. */
#include "BitMatrix.h"
#include "datamatrix/DMDetector.h"
#include "PerspectiveTransform.h"
#include <algorithm>
#include <array>
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
    constexpr uint8_t constants[] = {8, Step == 2 ? 6 : 0};
#ifdef CONFIG_IDF_TARGET_ESP32P4
    alignas(16) static constexpr auto vector_constants = [] {
        std::array<uint8_t, 32> values{};
        std::fill_n(values.begin(), 16, 8);
        std::fill_n(values.begin() + 16, 16, Step == 2 ? 6 : 0);
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

extern "C" int xcr_dm_body_finish(const Image* image, BarcodeResult* result, int step,
    void* state, const XcrDeadline* deadline, XcrDmBodyTrace* trace)
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
            if (length <= 0 && gray_candidates++ < 3)
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
    } catch (const std::exception&) {
        /* The fast path can still run if scratch allocation fails. */
    }
    return 0;
}


extern "C" int xcr_dm_body_decode(const Image* image, BarcodeResult* result, int step,
    const XcrDeadline* deadline, uint64_t* threshold_us)
{
    void* state = xcr_dm_body_prepare(image, step, deadline, threshold_us);
    int decoded = xcr_dm_body_finish(image, result, step, state, deadline, nullptr);
    xcr_dm_body_discard(state);
    return decoded;
}
